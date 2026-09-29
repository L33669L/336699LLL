"""记录消息数和距最近消息的墙钟时间；不将有消息等同于传感器健康。"""
import time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class SensorMonitor:
    def __init__(self, node):
        self.counts = {'scan': 0, 'odom': 0}
        self.last = {'scan': None, 'odom': None}
        self.before = self.counts.copy()
        self.subscriptions = [node.create_subscription(
            typ, '/' + name, lambda msg, key=name: self.receive(key), qos_profile_sensor_data)
            for name, typ in [('scan', LaserScan), ('odom', Odometry)]]

    def receive(self, key):
        self.counts[key] += 1
        self.last[key] = time.monotonic()

    def begin(self):
        self.before = self.counts.copy()

    def snapshot(self):
        now = time.monotonic()
        return {key: {'messages_during_navigation': self.counts[key] - self.before[key],
                      'last_message_age_sec': None if self.last[key] is None else now - self.last[key]}
                for key in self.counts}
