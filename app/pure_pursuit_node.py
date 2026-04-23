#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
import math
import numpy as np

from core.pid_controller import PIDController

class PurePursuitNode(Node):
    """纯跟踪(Pure Pursuit)节点"""
    
    def __init__(self):
        super().__init__('pure_pursuit')
        
        # 参数
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('control_frequency', 50.0)  # Hz
        
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.control_frequency = self.get_parameter('control_frequency').get_parameter_value().double_value
        
        # 状态
        self.have_path = False
        self.have_odom = False
        self.current_path = None
        self.current_path_index = 0
        
        # 当前位姿
        self.current_pos = np.array([0.0, 0.0, 0.0])  # [x, y, z]
        self.current_yaw = 0.0
        
        # PID控制参数（可选，用于更平滑的跟踪）
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.1
        self.pid = PIDController(self.kp, self.ki, self.kd, output_limit=1.0)
        
        # 创建订阅者
        self.path_sub = self.create_subscription(
            Path,
            '/planning/path',
            self.path_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_world',
            self.odom_callback,
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
        
        self.get_logger().info('Pure Pursuit Node started')
        self.get_logger().info(f'Lookahead distance: {self.lookahead_distance}m')
        self.get_logger().info(f'Max linear vel: {self.max_linear_vel}m/s')
        self.get_logger().info(f'Max angular vel: {self.max_angular_vel}rad/s')
    
    def path_callback(self, msg):
        """处理路径消息"""
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return
        
        self.current_path = msg
        self.current_path_index = 0
        self.have_path = True
        
        self.get_logger().info(f'Received new path with {len(msg.poses)} points')
    
    def odom_callback(self, msg):
        """处理里程计消息"""
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z
        
        # 从四元数提取yaw角
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # 转换为欧拉角（yaw）
        self.current_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        self.have_odom = True
    
    def find_lookahead_point(self):
        """
        找到lookahead距离处的目标点
        
        如果path上的点很稀疏，会在路径段之间进行插值，
        找到精确的lookahead距离点，而不是简单地选择最近的路径点
        """
        if not self.have_path or self.current_path is None:
            return None
        
        if self.current_path_index >= len(self.current_path.poses):
            return None
        
        # 从当前索引向前搜索，找到跨越lookahead距离的路径段
        for i in range(self.current_path_index, len(self.current_path.poses)):
            pose = self.current_path.poses[i].pose
            point = np.array([pose.position.x, pose.position.y, pose.position.z])
            dist = np.linalg.norm(point[:2] - self.current_pos[:2])
            
            # 如果找到距离刚好超过lookahead_distance的点
            if dist >= self.lookahead_distance:
                # 如果这是第一个点，直接使用
                if i == self.current_path_index:
                    self.current_path_index = i
                    return point
                
                # 否则，在前一个点和当前点之间进行插值
                prev_pose = self.current_path.poses[i - 1].pose
                prev_point = np.array([prev_pose.position.x, prev_pose.position.y, prev_pose.position.z])
                prev_dist = np.linalg.norm(prev_point[:2] - self.current_pos[:2])
                
                # 如果前一个点已经超过lookahead距离，使用前一个点
                if prev_dist >= self.lookahead_distance:
                    self.current_path_index = i - 1
                    return prev_point
                
                # 在前一个点和当前点之间插值，找到精确的lookahead距离点
                # 线段插值：P = P1 + t * (P2 - P1), t in [0, 1]
                segment_vec = point[:2] - prev_point[:2]
                segment_len = np.linalg.norm(segment_vec)
                
                if segment_len < 1e-6:
                    # 两点重合，直接使用当前点
                    self.current_path_index = i
                    return point
                
                # 计算插值参数t，使得插值点到机器人的距离等于lookahead_distance
                # 使用二分法或解析方法求解
                # 简化：假设沿线段线性插值，找到距离为lookahead_distance的点
                t = self._find_interpolation_parameter(
                    prev_point[:2], point[:2], self.current_pos[:2], self.lookahead_distance
                )
                
                if t is not None and 0 <= t <= 1:
                    # 插值得到精确的lookahead点
                    lookahead_point_2d = prev_point[:2] + t * segment_vec
                    lookahead_point = np.array([lookahead_point_2d[0], lookahead_point_2d[1], prev_point[2]])
                    self.current_path_index = i
                    return lookahead_point
                else:
                    # 插值失败，使用当前点
                    self.current_path_index = i
                    return point
        
        # 如果到达路径末尾，使用最后一个点
        if len(self.current_path.poses) > 0:
            last_pose = self.current_path.poses[-1].pose
            last_point = np.array([last_pose.position.x, last_pose.position.y, last_pose.position.z])
            dist_to_end = np.linalg.norm(last_point[:2] - self.current_pos[:2])
            
            if dist_to_end < 0.1:
                # 路径完成
                return None
            
            self.current_path_index = len(self.current_path.poses) - 1
            return last_point
        
        return None
    
    def _find_interpolation_parameter(self, p1, p2, robot_pos, target_dist):
        """
        在线段p1-p2上找到一点，使得该点到robot_pos的距离等于target_dist
        
        Args:
            p1, p2: 线段端点 (2D)
            robot_pos: 机器人位置 (2D)
            target_dist: 目标距离
            
        Returns:
            t: 插值参数 [0, 1]，如果无解返回None
        """
        # 线段向量
        seg_vec = p2 - p1
        seg_len = np.linalg.norm(seg_vec)
        
        if seg_len < 1e-6:
            return None
        
        # 归一化方向向量
        seg_dir = seg_vec / seg_len
        
        # 从p1到robot_pos的向量
        to_robot = robot_pos - p1
        
        # 将to_robot投影到线段方向上
        proj_len = np.dot(to_robot, seg_dir)
        
        # 垂直于线段的方向上的距离
        perp_vec = to_robot - proj_len * seg_dir
        perp_dist = np.linalg.norm(perp_vec)
        
        # 如果垂直距离大于target_dist，无解
        if perp_dist > target_dist:
            return None
        
        # 计算沿线段方向需要移动的距离
        # 使用勾股定理：target_dist^2 = perp_dist^2 + along_dist^2
        along_dist_sq = target_dist**2 - perp_dist**2
        if along_dist_sq < 0:
            return None
        
        along_dist = math.sqrt(along_dist_sq)
        
        # 计算插值参数t
        # 有两个解：t1 = (proj_len - along_dist) / seg_len
        #          t2 = (proj_len + along_dist) / seg_len
        # 选择在[0, 1]范围内且更接近p2的解
        t1 = (proj_len - along_dist) / seg_len
        t2 = (proj_len + along_dist) / seg_len
        
        # 选择在[0, 1]范围内的解，优先选择更接近1的（更接近p2）
        valid_ts = [t for t in [t1, t2] if 0 <= t <= 1]
        
        if not valid_ts:
            return None
        
        # 返回最接近1的解（更接近p2，即更前面的点）
        return max(valid_ts)
    
    def compute_pure_pursuit(self, lookahead_point):
        """计算Pure Pursuit控制指令"""
        # 计算到目标点的向量
        to_target = lookahead_point[:2] - self.current_pos[:2]
        dist_to_target = np.linalg.norm(to_target)
        
        if dist_to_target < 0.01:
            return 0.0, 0.0
        
        # 期望航向角
        desired_yaw = math.atan2(to_target[1], to_target[0])
        
        # 航向误差
        yaw_error = desired_yaw - self.current_yaw
        
        # 归一化到[-pi, pi]
        while yaw_error > math.pi:
            yaw_error -= 2.0 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2.0 * math.pi
        
        # Pure Pursuit方程
        # 线速度：与距离成比例（有最大限制）
        linear_vel = min(self.max_linear_vel * 0.5, dist_to_target * 0.5)
        
        # 角速度：考虑曲率
        curvature = 2.0 * math.sin(yaw_error) / self.lookahead_distance
        angular_vel = linear_vel * curvature
        
        # 如果航向误差较大，降低线速度，增加角速度
        if abs(yaw_error) > 0.5:
            linear_vel *= 0.5
            angular_vel = math.copysign(min(abs(angular_vel), self.max_angular_vel), angular_vel)
        
        return linear_vel, angular_vel
    
    def apply_pid_control(self, target_point, linear_vel, angular_vel):
        """应用PID控制进行更平滑的跟踪（可选）"""
        to_target_2d = target_point[:2] - self.current_pos[:2]
        dist = np.linalg.norm(to_target_2d)
        
        if dist < 0.01:
            return linear_vel, angular_vel
        
        # 计算横向误差（使用叉积）
        forward = np.array([math.cos(self.current_yaw), math.sin(self.current_yaw)])
        lateral_error = forward[0] * to_target_2d[1] - forward[1] * to_target_2d[0]
        
        # PID控制
        dt = 1.0 / self.control_frequency
        pid_output = self.pid.compute(lateral_error, dt)
        
        # 应用到角速度
        angular_vel += pid_output
        
        return linear_vel, angular_vel
    
    def control_callback(self):
        """控制回调函数"""
        if not self.have_path or not self.have_odom:
            return
        
        if self.current_path is None or len(self.current_path.poses) == 0:
            return
        
        # 找到lookahead点
        lookahead_point = self.find_lookahead_point()
        
        if lookahead_point is None:
            # 路径完成，停止
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.have_path = False
            self.get_logger().info('Path completed, stopping')
            return
        
        # 计算Pure Pursuit控制
        linear_vel, angular_vel = self.compute_pure_pursuit(lookahead_point)
        
        # 应用PID控制（可选，用于更平滑的跟踪）
        linear_vel, angular_vel = self.apply_pid_control(lookahead_point, linear_vel, angular_vel)
        
        # 限制速度
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        # 发布cmd_vel
        cmd = Twist()
        cmd.linear.x = float(linear_vel)
        cmd.angular.z = float(angular_vel)
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

