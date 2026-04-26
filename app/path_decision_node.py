#!/usr/bin/env python3
"""
路径决策节点

功能：
- 订阅odom_world、kfs_data和path话题
- 发布can_go信号（Bool类型）
- 初始为False，启动时发送一次True
- 当检测到下一个path点上是r2kfs时，发送False等待1秒再发送True
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import json
import numpy as np

from core.grid_utils import GridConverter


class PathDecisionNode(Node):
    """路径决策节点"""

    def __init__(self):
        super().__init__('path_decision')

        self.declare_parameter('grid.map_origin', [3.2, 1.2, 0.0])
        self.declare_parameter('grid.grid_rows', 4)
        self.declare_parameter('grid.grid_cols', 3)
        self.declare_parameter('grid.grid_resolution', 1.2)

        map_origin_raw = self.get_parameter('grid.map_origin').value
        self.MAP_ORIGIN = tuple(map_origin_raw)
        self.GRID_ROWS = self.get_parameter('grid.grid_rows').value
        self.GRID_COLS = self.get_parameter('grid.grid_cols').value
        self.GRID_RESOLUTION = self.get_parameter('grid.grid_resolution').value

        self.grid_converter = GridConverter(
            grid_rows=self.GRID_ROWS,
            grid_cols=self.GRID_COLS,
            map_origin=self.MAP_ORIGIN,
            grid_resolution=self.GRID_RESOLUTION
        )

        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.kfs_grid = np.zeros((self.GRID_ROWS, self.GRID_COLS), dtype=int)
        self.current_path = None

        self.can_go = False
        self.initialized = False
        self.used = np.zeros((self.GRID_ROWS, self.GRID_COLS), dtype=bool)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        odom_topic = self.declare_parameter('topics.odom_world', '/odom_world').value
        kfs_topic = self.declare_parameter('topics.kfs_grid_data', '/kfs_grid_data').value
        path_topic = self.declare_parameter('topics.planning_path', '/planning/path').value
        can_go_topic = self.declare_parameter('topics.can_go', '/can_go').value

        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )
        self.kfs_data_sub = self.create_subscription(
            String, kfs_topic, self.kfs_data_callback, qos_profile
        )
        self.path_sub = self.create_subscription(
            Path, path_topic, self.path_callback, qos_profile
        )

        self.can_go_pub = self.create_publisher(Bool, can_go_topic, 10)

        self.init_timer = self.create_timer(0.1, self.init_timer_callback)
        self.check_timer = self.create_timer(0.1, self.check_timer_callback)

        self.get_logger().info('Path Decision Node started')

    def odom_callback(self, msg):
        """处理里程计消息"""
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z

    def kfs_data_callback(self, msg):
        """处理kfs_grid_data消息"""
        try:
            grid_data = json.loads(msg.data)
            grid = np.array(grid_data['grid'])

            if grid.shape != (self.GRID_ROWS, self.GRID_COLS):
                self.get_logger().warn(f'Invalid grid dimensions: {grid.shape}')
                return

            self.kfs_grid = grid
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
        self.used = np.zeros((self.GRID_ROWS, self.GRID_COLS), dtype=bool)
        self.get_logger().info(f'Received path with {len(msg.poses)} points')

    def find_current_path_index(self) -> int:
        """找到当前位置对应的path索引（下一个未到达的点）"""
        if self.current_path is None:
            return 0

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

    def get_next_path_point(self) -> tuple:
        """获取下一个path点的grid坐标"""
        if self.current_path is None:
            return None

        current_index = self.find_current_path_index()
        next_index = current_index + 1

        if next_index >= len(self.current_path.poses):
            return None

        pose = self.current_path.poses[next_index].pose
        return self.grid_converter.map_to_grid(pose.position.x, pose.position.y)

    def check_r2_kfs(self) -> tuple:
        """检查下一个点是否是r2kfs且未被使用"""
        if self.current_path is None:
            return False, None, None

        next_point = self.get_next_path_point()
        if next_point is None:
            return False, None, None

        row, col = next_point

        if not self.grid_converter.is_in_grid(row, col):
            return False, None, None

        is_r2kfs = self.kfs_grid[row, col] == 2
        is_used = self.used[row, col]

        if is_r2kfs and not is_used:
            return True, row, col

        return False, None, None

    def init_timer_callback(self):
        """初始化定时器：启动时发送一次True"""
        if not self.initialized:
            if self.current_pos[0] != 0.0 and self.current_pos[1] != 0.0:
                self.can_go = True
                self.publish_can_go()
                self.initialized = True
                self.get_logger().info('Initialized: sent can_go=True')
                self.init_timer.cancel()

    def check_timer_callback(self):
        """检查定时器：检测r2kfs并控制can_go"""
        if not self.initialized:
            return

        is_r2kfs, row, col = self.check_r2_kfs()

        if is_r2kfs:
            if self.can_go:
                self.can_go = False
                self.publish_can_go()
                self.used[row, col] = True
                self.get_logger().info(f'Detected r2kfs at grid [{row}, {col}], sending can_go=False')
                self.wait_timer = self.create_timer(1.0, self.wait_timer_callback)
        else:
            if not hasattr(self, 'wait_timer') or not self.wait_timer.is_ready():
                if not self.can_go:
                    self.can_go = True
                    self.publish_can_go()
                    self.get_logger().info('No r2kfs detected, sending can_go=True')

    def wait_timer_callback(self):
        """等待定时器回调：1秒后发送True"""
        self.can_go = True
        self.publish_can_go()
        self.get_logger().info('Wait completed, sending can_go=True')
        if hasattr(self, 'wait_timer'):
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
