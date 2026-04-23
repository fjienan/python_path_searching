#!/usr/bin/env python3
"""
全向轮路径跟踪节点

功能：
- 订阅path、odom_world和kfs_data，使用PID控制跟踪路径
- 订阅/localization/pose，接受定位模块的实时位置
- 订阅/robot_distribution，发布机器人分布位置给其他模块
- 梯形速度规划：从格子中心开始加速，到下一个格子中心减速，重复完成整条路径

"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist, Pose
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import math
import numpy as np
import json
from math import floor

from core.pid_controller import PIDController
from core.transform_utils import euler_from_quaternion

class OmnidirectionalTrackerNode(Node):
    """全向轮路径跟踪节点"""

    def __init__(self):
        super().__init__('omnidirectional_tracker')

        # 参数（直接赋值）
        self.kp_linear = 5.0
        self.ki_linear = 0.0
        self.kd_linear = 0.1
        self.kp_angular = 10.0
        self.ki_angular = 0.0
        self.kd_angular = 0.1
        self.max_linear_vel = 0.5
        self.max_angular_vel = 5.0
        self.control_frequency = 50.0
        self.target_distance_threshold = 0.1
        self.kfs_check_distance = 0.7

        # 网格配置（与astar_planner一致）
        self.grid_rows = 4
        self.grid_cols = 3
        self.map2_origin = [3.2, 1.2, 0.0]
        self.grid_resolution = 1.2

        # 状态
        self.have_path = False
        self.have_odom = False
        self.have_kfs_data = False
        self.current_path = None
        self.current_path_index = 0

        # KFS网格数据
        self.kfs_grid = np.zeros((self.grid_rows, self.grid_cols), dtype=int)

        # 等待KFS标志
        self.is_waiting_for_kfs = False
        self.waiting_target = None

        # 当前位姿
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.current_yaw = 0.0

        # PID控制状态
        self.pid_x = PIDController(kp=1.0, ki=0.0, kd=0.1, output_limit=self.trap_max_vel)
        self.pid_y = PIDController(kp=1.0, ki=0.0, kd=0.1, output_limit=self.trap_max_vel)
        self.pid_yaw = PIDController(kp=1.0, ki=0.0, kd=0.1, output_limit=self.trap_max_vel)

        # 当前速度（用于平滑减速）
        self.current_vel_x = 0.0
        self.current_vel_y = 0.0
        self.current_vel_yaw = 0.0

        # 梯形速度规划参数（格子间距1.2米）
        self.trape_accel_distance = 0.4  # 加速距离（前1/3）
        self.trap_decel_distance = 0.4  # 减速距离（后1/3）
        self.trap_max_vel = 0.3  # 梯形最大速度
        self.trap_max_accel = 1.0  # 梯形最大加速度

        # 配置 QoS
        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # 订阅路径
        self.path_sub = self.create_subscription(
            Path,
            '/path_planning/path',
            self.path_callback,
            path_qos
        )

        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_world',
            self.odom_callback,
            10
        )

        # 订阅KFS数据
        kfs_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.kfs_data_sub = self.create_subscription(
            String,
            '/kfs_grid_data',
            self.kfs_data_callback,
            kfs_qos
        )

        # 订阅定位模块位置
        localization_pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.localization_pose_sub = self.create_subscription(
            PoseStamped,
            '/localization/pose',
            self.localization_pose_callback,
            localization_pose_qos
        )

        # 发布机器人分布位置
        self.robot_distribution_pub = self.create_publisher(
            PoseStamped,
            '/robot_distribution',
            10
        )

        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 创建控制定时器
        control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(control_period, self.control_callback)

        self.get_logger().info('Omnidirectional Tracker Node started')
        self.get_logger().info(f'Max linear vel: {self.max_linear_vel}m/s')
        self.get_logger().info(f'Max angular vel: {self.max_angular_vel}rad/s')
        self.get_logger().info(f'KFS check distance: {self.kfs_check_distance}m')

    def localization_pose_callback(self, msg):
        """
        处理定位模块的实时位置消息

        Args:
            msg: PoseStamped 实时位置
        """
        # 更新当前位置和偏航角
        self.current_pos[0] = msg.pose.position.x
        self.current_pos[1] = msg.pose.position.y
        self.current_pos[2] = msg.pose.position.z

        # 从四元数提取偏航角
        if msg.pose.orientation.w < 1.0:
            self.current_yaw = math.atan2(2.0 * msg.pose.orientation.w, 0.0)
        else:
            siny_cosp = 2.0 * (msg.pose.orientation.w * msg.pose.orientation.z + msg.pose.orientation.x * msg.pose.orientation.y)
            cosy_cosp = 1.0 - 2.0 * (msg.pose.orientation.y * msg.pose.orientation.y + msg.pose.orientation.z * msg.pose.orientation.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # 发布分布位置
        self.publish_robot_distribution()

    def path_callback(self, msg):
        """处理路径消息"""
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return

        self.current_path = msg
        self.have_path = True

        # 重置等待状态
        self.is_waiting_for_kfs = False
        self.waiting_target = None
        self.current_path_index = 0

        self.get_logger().info(f'Received path with {len(msg.poses)} points')

    def odom_callback(self, msg):
        """里程计回调函数"""
        # 更新当前位姿
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z

        # 从四元数计算偏航角
        _, _, self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation)

        self.have_odom = True

    def kfs_data_callback(self, msg):
        """处理KFS网格数据消息"""
        try:
            grid_data = json.loads(msg.data)
            grid = np.array(grid_data['grid'])

            if grid.shape != (self.grid_rows, self.grid_cols):
                self.get_logger().warn(f'Invalid grid dimensions: {grid.shape}')
                return

            self.kfs_grid = grid
            self.have_kfs_data = True

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing kfs_data: {e}')

    def publish_robot_distribution(self):
        """发布机器人分布位置到/robot_distribution话题"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.current_pos[0]
        pose.pose.position.y = self.current_pos[1]
        pose.pose.position.z = self.current_pos[2]

        # 使用简化的四元数表示偏航角
        pose.pose.orientation.w = math.cos(self.current_yaw / 2.0)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0

        self.robot_distribution_pub.publish(pose)

    def control_callback(self):
        """控制回调：计算速度命令"""
        if not self.have_path:
            return

        # 获取当前目标点
        target_pose = self.current_path.poses[self.current_path_index]

        # 计算到目标点的距离
        dx = target_pose.pose.position.x - self.current_pos[0]
        dy = target_pose.pose.position.y - self.current_pos[1]
        dist = math.sqrt(dx**2 + dy**2)

        # 如果到达目标点，移动到下一个
        if dist < self.target_distance_threshold:
            self.current_path_index += 1

            if self.current_path_index >= len(self.current_path.poses):
                return

        # 使用梯形速度规划
        velocity_x, velocity_y, direction = self.trap_velocity_control(target_pose)

        # 创建速度命令
        cmd = Twist()
        cmd.linear.x = float(velocity_x)
        cmd.linear.y = float(velocity_y)
        cmd.angular.z = float(direction)

        self.cmd_vel_pub.publish(cmd)

    def trap_velocity_control(self, target_pose):
        """
        梯形速度控制：从格子中心开始加速，到下一个格子中心减速，重复完成整条路径

        Args:
            target_pose: 目标位姿

        Returns:
            tuple: (velocity_x, velocity_y, direction) 速度和方向
        """
        # 计算目标点坐标
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y

        # 计算当前点到目标点的距离（物理坐标）
        dx = target_x - self.current_pos[0]
        dy = target_y - self.current_pos[1]
        distance = math.sqrt(dx**2 + dy**2)

        # 到目标点的航向
        target_yaw = math.atan2(dy, dx)

        # 当前航向到目标航向的角度差
        angle_diff = target_yaw - self.current_yaw

        # 归一化角度差到 [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2.0 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2.0 * math.pi

        # 初始化速度
        velocity_x = 0.0
        velocity_y = 0.0

        # 梯形速度规划
        # 加速距离：当距离大于此值时，加速到最大速度
        # 减速距离：当距离小于此值时，开始减速
        max_vel = self.trap_max_vel

        if distance > self.trap_accel_distance:
            # 加速阶段：使用最大速度
            vel = max_vel
        elif distance > self.trap_decel_distance:
            # 匀速阶段：保持最大速度
            vel = max_vel
        else:
            # 减速阶段：速度随距离线性减小
            vel = max_vel * (distance / self.trap_decel_distance)

        # 限制最小速度
        vel = max(vel, 0.05)

        # 角度补偿：根据角度差调整速度
        angle_factor = math.cos(angle_diff)
        vel = vel * angle_factor

        # 速度分解到x和y方向
        velocity_x = vel * math.cos(target_yaw)
        velocity_y = vel * math.sin(target_yaw)

        # 方向：角度差
        direction = angle_diff

        return velocity_x, velocity_y, direction

    def map_to_grid_coords(self, x, y):
        """将map坐标转换为grid坐标"""
        from math import floor
        row = floor((x - self.map2_origin[0]) / self.grid_resolution)
        col = floor((y - self.map2_origin[1]) / self.grid_resolution)
        return row, col


def main(args=None):
    rclpy.init(args=args)
    node = OmnidirectionalTrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()
