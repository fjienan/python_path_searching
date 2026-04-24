"""
坐标变换工具

提供四元数与欧拉角之间的转换功能
"""

import math
import numpy as np


def euler_from_quaternion(quaternion):
    """
    将四元数转换为欧拉角 (roll, pitch, yaw)

    Args:
        quaternion: 四元数 [x, y, z, w] 或包含 x,y,z,w 属性的对象

    Returns:
        tuple: (roll, pitch, yaw) 弧度制的欧拉角
    """
    if hasattr(quaternion, 'x'):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    else:
        x, y, z, w = quaternion[0], quaternion[1], quaternion[2], quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    将欧拉角转换为四元数

    Args:
        roll: roll角（绕x轴），弧度
        pitch: pitch角（绕y轴），弧度
        yaw: yaw角（绕z轴），弧度

    Returns:
        ndarray: 四元数 [x, y, z, w]
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return np.array([qx, qy, qz, qw])


def normalize_angle(angle: float) -> float:
    """
    将角度归一化到 [-pi, pi] 范围内

    Args:
        angle: 输入角度（弧度）

    Returns:
        float: 归一化后的角度
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(quaternion) -> float:
    """
    从四元数提取yaw角

    Args:
        quaternion: 四元数

    Returns:
        float: yaw角（弧度）
    """
    _, _, yaw = euler_from_quaternion(quaternion)
    return yaw
