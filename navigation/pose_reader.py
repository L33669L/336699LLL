"""TF/Gazebo 最终采样；不是严格同步的动态精度测量。"""
import math
import subprocess
import time
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from .ground_truth import ground_truth
from .metrics import quaternion_to_yaw
from .result import Pose2D


class PoseReader:
    def __init__(self, node, config):
        self.node, self.config = node, config
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)

    def capture(self, result):
        c = self.config
        result.sampling['synchronized'] = False
        result.sampling['frame_assumption'] = '默认 baseline 的 Gazebo world 与 map 对齐'
        start = time.monotonic()
        try:
            truth = ground_truth(c.gazebo_timeout_sec, c.gazebo_topic, c.gazebo_model)
            result.gazebo_pose = Pose2D(truth['x'], truth['y'], math.degrees(truth['yaw']))
            self.node.get_logger().info(f'Gazebo真实位姿：{result.gazebo_pose}')
        except (OSError, subprocess.SubprocessError, ValueError) as exc:
            result.errors.append(f'Gazebo真值读取失败：{exc}')
        result.sampling['gazebo_read_sec'] = time.monotonic() - start
        # gz 子进程期间本节点未处理回调，先刷新 TF，保留原来的采样顺序。
        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=c.spin_period_sec)
        deadline = time.monotonic() + c.tf_timeout_sec
        while rclpy.ok():
            try:
                tf = self.buffer.lookup_transform(c.map_frame, c.base_frame, Time())
                p, q = tf.transform.translation, tf.transform.rotation
                result.tf_pose = Pose2D(p.x, p.y, quaternion_to_yaw(q.x, q.y, q.z, q.w))
                result.sampling['tf_stamp_ros_sec'] = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
                result.sampling['tf_clock_delta_sec'] = (self.node.get_clock().now().nanoseconds / 1e9
                                                        - result.sampling['tf_stamp_ros_sec'])
                self.node.get_logger().info(f'TF位姿：{result.tf_pose}')
                break
            except (TransformException, ValueError) as exc:
                if time.monotonic() >= deadline:
                    result.errors.append(f'TF位姿读取失败：{exc}')
                    break
                rclpy.spin_once(self.node, timeout_sec=c.spin_period_sec)
        result.sampling['total_read_sec'] = time.monotonic() - start
        result.calculate_metrics()
