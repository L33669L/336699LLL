"""真实 ROS2 Action 通信测试：独立假服务器，不发送机器人运动命令。"""
from dataclasses import replace
from threading import Event, Thread
import time
import uuid
import pytest

pytestmark = pytest.mark.integration


def test_action_timeout_cancel_over_real_ros():
    import rclpy
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from navigation.config import NavigationConfig
    from navigation.navigator import Navigator
    from navigation.result import Pose2D

    rclpy.init()
    server_node = rclpy.create_node('p2_controlled_test_server')
    stopped = Event()
    def execute(handle):
        while not stopped.wait(0.02):
            if handle.is_cancel_requested:
                handle.canceled()
                return NavigateToPose.Result()
        handle.abort()
        return NavigateToPose.Result()
    name = '/p2_test_action_' + uuid.uuid4().hex
    server = ActionServer(server_node, NavigateToPose, name, execute_callback=execute,
                          cancel_callback=lambda req: CancelResponse.ACCEPT,
                          callback_group=ReentrantCallbackGroup())
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server_node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    node = None
    try:
        node = Navigator(replace(NavigationConfig(), action_name=name, warmup_sec=0.1,
                                 settle_sec=0.1, navigation_timeout_sec=0.2))
        # 本测试只验证通信清理，不能将假服务器结果当成导航/采样通过。
        node.pose_reader.capture = lambda report: None
        start = time.monotonic()
        report = node.run(Pose2D(0, 0, 0))
        assert report.outcome == 'navigation_timeout'
        assert report.cancel_requested and report.cancel_acknowledged
        assert report.terminal_confirmed
        assert report.action_status == GoalStatus.STATUS_CANCELED
        assert not report.errors, report.errors
        assert time.monotonic() - start < 10
    finally:
        stopped.set()
        if node is not None:
            node.destroy_node()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        server.destroy()
        server_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

