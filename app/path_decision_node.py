#!/usr/bin/env python3
"""
路径决策节点

功能：
- 订阅odom_world、kfs_data和path
- 发布can_go信号（Bool类型）
- 初始为False，启动时发送一次True
- 当检测到机器人当前位置对应的下一个path点的下一个path点上的kfs_data是r2kfs（值为2）时，
  发送False，等待1秒再发送True
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import json
import numpy as np
import math


class PathDecisionNode(Node):
    """路径决策节点"""
    
    def __init__(self):
        super().__init__('path_decision')
        
        # 网格配置（与astar_planner一致）
        self.grid_rows = 4
        self.grid_cols = 3
        
        # Map2配置（用于坐标转换）
        self.map2_origin = [3.2, 1.2, 0.0]  # [x, y, theta]
        self.grid_resolution = 1.2  # 米/单元格
        
        # 数据标志
        self.have_odom = False
        self.have_kfs_data = False
        self.have_path = False
        
        # 存储数据
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.kfs_grid = np.zeros((self.grid_rows, self.grid_cols), dtype=int)
        self.current_path = None
        
        # can_go状态
        self.can_go = False
        self.initialized = False
        
        # 标记已经停下过的点（used数组）
        self.used = np.zeros((self.grid_rows, self.grid_cols), dtype=bool)
        
        # 配置QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 创建订阅者
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_world',
            self.odom_callback,
            10
        )
        
        self.kfs_data_sub = self.create_subscription(
            String,
            '/kfs_grid_data',
            self.kfs_data_callback,
            qos_profile
        )
        
        # 创建订阅者（使用TRANSIENT_LOCAL QoS）
        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_sub = self.create_subscription(
            Path,
            '/planning/path',
            self.path_callback,
            path_qos
        )
        
        # 创建发布者
        self.can_go_pub = self.create_publisher(
            Bool,
            '/can_go',
            10
        )
        
        # 创建定时器用于初始化和检查
        self.init_timer = self.create_timer(0.1, self.init_timer_callback)
        self.check_timer = self.create_timer(0.1, self.check_timer_callback)
        
        self.get_logger().info('Path Decision Node started')
    
    def odom_callback(self, msg):
        """处理里程计消息"""
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z
        self.have_odom = True
    
    def kfs_data_callback(self, msg):
        """处理kfs_grid_data消息"""
        try:
            grid_data = json.loads(msg.data)
            grid = np.array(grid_data['grid'])
            
            if grid.shape != (self.grid_rows, self.grid_cols):
                self.get_logger().warn(f'Invalid grid dimensions: {grid.shape}')
                return
            
            self.kfs_grid = grid
            self.have_kfs_data = True
            
            self.get_logger().info(f'Received kfs_data:\n{grid}')
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing kfs_data: {e}')
    
    def path_callback(self, msg):
        """处理路径消息"""
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return
        
        self.current_path = msg
        self.have_path = True
        # 重置used数组（新路径时清除标记）
        self.used = np.zeros((self.grid_rows, self.grid_cols), dtype=bool)
        self.get_logger().info(f'Received path with {len(msg.poses)} points')
    
    def map_to_grid_coords(self, x, y):
        """
        将map坐标转换为grid索引
        
        Args:
            x, y: map坐标
            
        Returns:
            tuple: (row, col) grid索引
        """
        row = int((x - self.map2_origin[0]) / self.grid_resolution)
        col = int((y - self.map2_origin[1]) / self.grid_resolution)
        return row, col
    
    def find_current_path_index(self):
        """
        找到当前位置对应的path索引（下一个未到达的点）
        
        找到距离当前位置最近且还未到达的path点
        """
        if not self.have_path or self.current_path is None:
            return None
        
        # 找到距离当前位置最近的path点
        min_dist = float('inf')
        min_index = 0
        
        for i, pose_stamped in enumerate(self.current_path.poses):
            pose = pose_stamped.pose
            point = np.array([pose.position.x, pose.position.y])
            dist = np.linalg.norm(point - self.current_pos[:2])
            
            if dist < min_dist:
                min_dist = dist
                min_index = i
        
        
        return min_index
    
    def get_next_path_point(self):
        """
        获取下一个path点
        
        Returns:
            tuple: (row, col) grid坐标，如果不存在返回None
        """
        if not self.have_path or self.current_path is None:
            return None
        
        current_index = self.find_current_path_index()
        if current_index is None:
            return None
        
        # 下一个点的索引
        next_index = current_index + 1
        if next_index >= len(self.current_path.poses):
            return None

        
        # 获取下一个点的下一个点的坐标
        pose = self.current_path.poses[next_index].pose
        x = pose.position.x
        y = pose.position.y
        
        # 转换为grid坐标
        row, col = self.map_to_grid_coords(x, y)
        
        return row, col
    
    def check_r2_kfs(self):
        """
        检查下一个点是否是r2kfs（值为2）且未被标记为已使用
        
        Returns:
            tuple: (is_r2kfs, row, col) 如果是r2kfs且未使用返回(True, row, col)，否则返回(False, None, None)
        """
        if not self.have_odom or not self.have_kfs_data or not self.have_path:
            return False, None, None
        
        next_point = self.get_next_path_point()
        if next_point is None:
            return False, None, None
        
        row, col = next_point
        
        # 检查grid坐标是否有效
        if row < 0 or row >= self.grid_rows or col < 0 or col >= self.grid_cols:
            return False, None, None
        
        # 检查是否是r2kfs（值为2）且未被使用
        is_r2kfs = self.kfs_grid[row, col] == 2
        is_used = self.used[row, col]
        
        if is_r2kfs and not is_used:
            return True, row, col
        
        return False, None, None
    
    def init_timer_callback(self):
        """初始化定时器：启动时发送一次True"""
        if not self.initialized:
            # 等待所有数据准备好
            if self.have_odom and self.have_kfs_data:
                self.can_go = True
                self.publish_can_go()
                self.initialized = True
                self.get_logger().info('Initialized: sent can_go=True')
                self.init_timer.cancel()
    
    def check_timer_callback(self):
        """检查定时器：检测r2kfs并控制can_go"""
        if not self.initialized:
            return
        
        # 检查下一个点是否是r2kfs且未被使用
        is_r2kfs, row, col = self.check_r2_kfs()
        
        if is_r2kfs:
            if self.can_go:
                # 发现r2kfs且未使用，发送False并标记为已使用
                self.can_go = False
                self.publish_can_go()
                # 标记该点为已使用
                self.used[row, col] = True
                self.get_logger().info(f'Detected r2kfs at grid [{row}, {col}], sending can_go=False and marking as used')
                
                # 创建单次定时器，1秒后发送True
                self.wait_timer = self.create_timer(1.0, self.wait_timer_callback)
        else:
            # 没有r2kfs或已经使用过，确保can_go为True（除非正在等待）
            if not hasattr(self, 'wait_timer') or not self.wait_timer.is_ready():
                if not self.can_go:
                    self.can_go = True
                    self.publish_can_go()
                    self.get_logger().info('No r2kfs detected or already used, sending can_go=True')
    
    def wait_timer_callback(self):
        """等待定时器回调：1秒后发送True"""
        self.can_go = True
        self.publish_can_go()
        self.get_logger().info('Wait completed, sending can_go=True')
        self.wait_timer.cancel()
        delattr(self, 'wait_timer')
    
    def publish_can_go(self):
        """发布can_go信号"""
        msg = Bool()
        msg.data = self.can_go
        self.can_go_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathDecisionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
