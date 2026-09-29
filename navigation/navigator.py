"""一次导航的有界执行；耗时均使用 monotonic，不依赖仿真时钟推进。"""
import math
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from unique_identifier_msgs.msg import UUID

from .result import NavigationResult
from .pose_reader import PoseReader
from .sensors import SensorMonitor

TERMINAL = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED}
STATUS_NAMES = {getattr(GoalStatus, name): name.removeprefix('STATUS_')
                for name in dir(GoalStatus) if name.startswith('STATUS_')}


class Navigator(Node):
    def __init__(self, config):
        super().__init__('navigation_goal_sender',
                         parameter_overrides=[Parameter('use_sim_time', value=config.use_sim_time)])
        self.config = config
        self.action_client = ActionClient(self, NavigateToPose, config.action_name)
        # 目标应答丢失时，也能按本次 UUID 取消，绝不取消其他客户端的目标。
        self.cancel_client = self.create_client(CancelGoal, config.action_name + '/_action/cancel_goal')
        self.pose_reader = PoseReader(self, config)
        self.sensors = SensorMonitor(self)
        self.goal_handle = self.result_future = self.send_future = None
        self.goal_uuid = None

    def _wait(self, future, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            rclpy.spin_once(self, timeout_sec=min(self.config.spin_period_sec, remaining))
        return future.done()

    def _spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(self.config.spin_period_sec,
                                                max(0.0, deadline - time.monotonic())))

    def _record_terminal(self, report):
        rr = self.result_future.result()
        report.action_status = rr.status
        report.action_status_name = STATUS_NAMES.get(rr.status, 'UNKNOWN')
        report.terminal_confirmed = rr.status in TERMINAL
        report.action_error_code = getattr(rr.result, 'error_code', None)
        report.action_error_message = getattr(rr.result, 'error_msg', '')
        self.get_logger().info(f'导航任务状态：{report.action_status_name}')

    def _cancel(self, report):
        if self.send_future is None or report.terminal_confirmed:
            return
        report.cancel_requested = True
        self.get_logger().warning('正在取消本次导航目标...')
        try:
            if self.goal_handle is None:
                # 给迟到的目标应答一个有界宽限期；不能取消 future 并遗忘服务器目标。
                if self._wait(self.send_future, self.config.cancel_timeout_sec):
                    self.goal_handle = self.send_future.result()
                    if not self.goal_handle.accepted:
                        report.cancel_requested = False
                        return
                    self.result_future = self.goal_handle.get_result_async()
            if self.goal_handle is not None:
                future = self.goal_handle.cancel_goal_async()
            else:
                if not self.cancel_client.wait_for_service(timeout_sec=self.config.cancel_timeout_sec):
                    report.errors.append('取消服务不可用；本次目标状态未知')
                    return
                req = CancelGoal.Request()
                req.goal_info.goal_id = self.goal_uuid
                future = self.cancel_client.call_async(req)
            if self._wait(future, self.config.cancel_timeout_sec):
                response = future.result()
                report.cancel_acknowledged = any(
                    list(g.goal_id.uuid) == list(self.goal_uuid.uuid)
                    for g in response.goals_canceling)
            if self.result_future is not None and self._wait(
                    self.result_future, self.config.cancel_result_timeout_sec):
                self._record_terminal(report)
            if not report.terminal_confirmed:
                report.errors.append('未确认 Action 最终状态；不能断言机器人已经停止')
        except Exception as exc:
            # 外部 ROS 通信边界：保留具体异常类型，而非伪装取消成功。
            report.errors.append(f'取消失败：{type(exc).__name__}: {exc}')

    def run(self, goal):
        self.goal_handle = self.result_future = self.send_future = None
        self.goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        report = NavigationResult(goal=goal, config=self.config.to_dict(),
                                  goal_id=bytes(self.goal_uuid.uuid).hex())
        start = time.monotonic()
        self.sensors.begin()
        try:
            self._spin_for(self.config.warmup_sec)
            self.get_logger().info('正在等待导航服务器...')
            if not self.action_client.wait_for_server(timeout_sec=self.config.server_timeout_sec):
                report.outcome = 'server_timeout'
                report.timeout_stage = 'server'
                return report
            msg = NavigateToPose.Goal()
            msg.pose.header.frame_id = self.config.map_frame
            msg.pose.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x, msg.pose.pose.position.y = float(goal.x), float(goal.y)
            radians = math.radians(goal.yaw_deg)
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(radians/2), math.cos(radians/2)
            self.sensors.begin()
            self.get_logger().info(f'已创建导航目标：{goal}')
            self.send_future = self.action_client.send_goal_async(msg, goal_uuid=self.goal_uuid)
            if not self._wait(self.send_future, self.config.goal_response_timeout_sec):
                report.outcome, report.timeout_stage = 'goal_response_timeout', 'goal_response'
                self._cancel(report)
                return report
            self.goal_handle = self.send_future.result()
            if not self.goal_handle.accepted:
                report.outcome = 'rejected'
                self.get_logger().warning('导航目标被拒绝')
                return report
            self.get_logger().info('导航目标已接受')
            self.result_future = self.goal_handle.get_result_async()
            if not self._wait(self.result_future, self.config.navigation_timeout_sec):
                report.outcome, report.timeout_stage = 'navigation_timeout', 'navigation'
                self._cancel(report)
            else:
                self._record_terminal(report)
                report.outcome = {
                    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                    GoalStatus.STATUS_ABORTED: 'aborted',
                    GoalStatus.STATUS_CANCELED: 'canceled',
                }.get(report.action_status, 'unexpected_status')
            # 消息统计在最终采样前定格，避免把 gz 阻塞时间算进导航数据新鲜度。
            report.sensors = self.sensors.snapshot()
            if report.terminal_confirmed:
                self.get_logger().info(f'等待 {self.config.settle_sec} 秒后进行最终采样')
                self._spin_for(self.config.settle_sec)
                self.pose_reader.capture(report)
        except KeyboardInterrupt:
            report.outcome = 'interrupted'
            self._cancel(report)
        except Exception as exc:
            report.outcome = 'error'
            report.errors.append(f'{type(exc).__name__}: {exc}')
            self._cancel(report)
        finally:
            report.elapsed_sec = time.monotonic() - start
            if not report.sensors:
                report.sensors = self.sensors.snapshot()
            for error in report.errors:
                self.get_logger().error(error)
        return report

