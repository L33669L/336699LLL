"""纯数学函数：位置单位 m，朝向和朝向误差单位 degree。"""
import math


def finite(*values):
    if not all(math.isfinite(v) for v in values):
        raise ValueError('位姿/误差输入必须是有限数值')


def position_error(x1, y1, x2, y2):
    finite(x1, y1, x2, y2)
    return math.hypot(x1 - x2, y1 - y2)


def yaw_error(yaw1, yaw2):
    finite(yaw1, yaw2)
    return abs((yaw1 - yaw2 + 180.0) % 360.0 - 180.0)


def quaternion_to_yaw(x, y, z, w):
    finite(x, y, z, w)
    norm = math.hypot(x, y, z, w)
    if norm == 0:
        raise ValueError('四元数不能为零')
    x, y, z, w = (v / norm for v in (x, y, z, w))
    return math.degrees(math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)))
