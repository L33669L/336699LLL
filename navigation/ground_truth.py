"""复用原 gz topic 采集方式，独立于 initial_pose 和 ROS2。"""
import math
import re
import subprocess
from .metrics import quaternion_to_yaw


def parse_ground_truth(raw, model='burger'):
    match = re.search(r'pose\s*\{\s*name:\s*"' + re.escape(model)
                      + r'".*?position\s*\{(.*?)\}\s*orientation\s*\{(.*?)\}', raw, re.S)
    if not match:
        raise ValueError(f'Gazebo 中未找到模型 {model} 的位姿')
    def fields(s):
        return {k: float(v) for k, v in re.findall(r'([xyzw]):\s*([-+\deE.]+)', s)}
    p, q = map(fields, match.groups())
    # protobuf 文本省略的分量均为零，包括四元数 w。
    yaw = quaternion_to_yaw(q.get('x', 0), q.get('y', 0), q.get('z', 0), q.get('w', 0))
    return dict(x=p.get('x', 0), y=p.get('y', 0), yaw=math.radians(yaw))


def ground_truth(timeout=20.0, topic='/world/default/dynamic_pose/info', model='burger'):
    raw = subprocess.check_output(['gz', 'topic', '-e', '-n', '1', '-t', topic],
                                  timeout=timeout, text=True)
    return parse_ground_truth(raw, model)
