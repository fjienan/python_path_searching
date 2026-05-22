#!/usr/bin/env python3
"""
DFS路径规划节点 - 简化版

功能：
- 仅订阅 kfs_data 话题
- 将地图上每个位置的kfs值传递给DFSPlanner处理
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float32, Float32MultiArray, String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import json
import numpy as np

# 假设你的DFSPlanner写在 core.dfs 中
from core.dfs import DFSPlanner
from core.step import Step

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import json
import math
import numpy as np

from core.dfs import DFSPlanner
from core.step import Step
from core.grid_utils import GridConverter

class DFSPlannerNode(Node):
    """DFS路径规划节点"""

    def __init__(self):
        super().__init__('dfs_planner')

        # 从参数服务器读取参数
        self.declare_parameter('grid_rows', 4)
        self.declare_parameter('grid_cols', 3)
        self.declare_parameter('start_row', -1)
        self.declare_parameter('start_col', 0)
        self.declare_parameter('reserved_length', 0.15)
        
        self.declare_parameter('initial_pose', '/initial_pose')

        self.GRID_ROWS = int(self.get_parameter('grid_rows').value)
        self.GRID_COLS = int(self.get_parameter('grid_cols').value)
        self.START_ROW = int(self.get_parameter('start_row').value)
        self.START_COL = int(self.get_parameter('start_col').value)
        self.initial_pos_x, self.initial_pos_y = None, None
        self.reserved_length = self.get_parameter('reserved_length').value

        # AStar中的参数注入与GridConverter初始化
        self.declare_parameter('map_origin', [3.2, 1.2, 0.0])
        self.declare_parameter('grid_resolution', 1.2)
        self.declare_parameter('team', 'red')

        map_origin_raw = self.get_parameter('map_origin').value
        self.MAP_ORIGIN = tuple(map_origin_raw)
        self.GRID_RESOLUTION = self.get_parameter('grid_resolution').value
        mirror_y = self.get_parameter('team').value.strip().lower() == 'blue'

        self.grid_converter = GridConverter(
            grid_rows=self.GRID_ROWS,
            grid_cols=self.GRID_COLS,
            map_origin=self.MAP_ORIGIN,
            grid_resolution=self.GRID_RESOLUTION,
            mirror_y=mirror_y,
        )

        # 初始化外部的DFS规划器
        self.planner = None
        self.path = None

        self.kfs_grid = np.zeros((self.GRID_ROWS, self.GRID_COLS), dtype=int)
        self.kfs_grid_height1 = np.array([
            [400, 200, 400],
            [200, 400, 600],
            [400, 600, 400],
            [200, 400, 200]
        ], dtype=int)
        self.kfs_grid_height2 = np.array([
            [400, 200, 400],
            [600, 400, 200],
            [400, 600, 400],
            [200, 400, 200]
        ], dtype=int)

        kfs_topic = self.declare_parameter('kfs_grid_data', '/kfs_grid_data').value
        path_topic = self.declare_parameter('planning_path', '/planning/path').value
        path_rviz_topic = self.declare_parameter('path_on_rviz', '/path_on_rviz').value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.kfs_data_sub = self.create_subscription(
            String, kfs_topic, self.kfs_data_callback, qos_profile
        )

        # 从网格入口计算初始位置（不再依赖 relocation）
        entry_x, entry_y = self.grid_converter.grid_to_map(self.START_ROW, self.START_COL)
        self.initial_pos_x = entry_x
        self.initial_pos_y = entry_y
        self.get_logger().info(f'Grid entry position: x={entry_x:.3f}, y={entry_y:.3f} (from start_row={self.START_ROW}, start_col={self.START_COL})')
        
        # Path 发布器
        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(String, path_topic, path_qos)
        self.path_rviz_pub = self.create_publisher(Path, path_rviz_topic, path_qos)

        self.get_logger().info('DFS Planner Node started (Simplified)')

    def kfs_data_callback(self, msg):
        """处理kfs_grid_data消息"""
        try:
            grid_data = json.loads(msg.data)
            grid = np.array(grid_data['grid'])

            if grid.shape != (self.GRID_ROWS, self.GRID_COLS):
                self.get_logger().warn(f'Invalid grid dimensions: {grid.shape}')
                return

            self.kfs_grid = grid
            self.kfs_data_received = True
            self.get_logger().info(f'Received kfs_data:\n{grid}')
            
            if self.planner is None: self._try_plan_path()

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing kfs_data: {e}')
    
    def _try_plan_path(self):
        """尝试初始化规划器并开始DFS路径规划，需要确保initial_pose和kfs_data都已收到"""
        if self.initial_pos_x is None or self.initial_pos_y is None:
            self.get_logger().warn('Initial position not set yet, waiting...')
            return
            
        if not getattr(self, 'kfs_data_received', False):
            self.get_logger().info('kfs_data not received yet, waiting...')
            return
        
        self.get_logger().info('Starting DFS path planning...')
            
        if self.planner is None:
            self.planner = DFSPlanner(self.GRID_COLS, self.GRID_ROWS, self.initial_pos_x, self.initial_pos_y, self.reserved_length, self.grid_converter, self.kfs_grid_height1, logger=self.get_logger())

        self.path = self.planner.plan_path(grid=self.kfs_grid, stx=self.START_ROW, sty=self.START_COL)
        if self.path is None:
            self.get_logger().warn(f'Empty path found.')
            self.delete_path()
        else:
            self.get_logger().info(f'Planned path with {len(self.path[0])} steps, total cost: {self.path[1]}, get {self.path[2]} kfs.')
            if len(self.path[3]) > 0: self.get_logger().warn(f'You have to remove kfs1 at {self.path[3]} first before launching R2')
            self.get_logger().info(f'Path details:')
            for i in range(len(self.path[0])):
                step = self.path[0][i]
                self.get_logger().info(f'  Step {i+1}/{len(self.path[0])}: x={step.x}, y={step.y}, yaw={math.degrees(step.yaw)}, require_can_go={step.require_can_go}, send_can_do={step.send_can_do}')
            
            # 发布路径
            self.publish_path(self.path[0])
    
    def delete_path(self):
        """删除旧路径（发布一个空的JSON路径和RViz Path）"""
        msg = String()
        msg.data = json.dumps({"points": []})
        self.path_pub.publish(msg)

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_rviz_pub.publish(path_msg)

    def publish_path(self, steps):
        """发布路径数据（JSON打包版和RViz Path版）"""
        self.delete_path()

        data = {
            "points": []
        }

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        if not steps:
            msg = String()
            msg.data = json.dumps(data)
            self.path_pub.publish(msg)
            self.path_rviz_pub.publish(path_msg)
            return

        for step in steps:
            point = {
                "x": float(step.x),
                "y": float(step.y),
                "yaw": float(step.yaw),
                "require_can_go": float(step.require_can_go),
                "send_can_do": float(step.send_can_do)
            }

            data["points"].append(point)

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(step.x)
            pose.pose.position.y = float(step.y)
            pose.pose.orientation.w = math.cos(float(step.yaw) * 0.5)
            pose.pose.orientation.z = math.sin(float(step.yaw) * 0.5)
            path_msg.poses.append(pose)

        msg = String()
        msg.data = json.dumps(data)

        self.path_pub.publish(msg)
        self.path_rviz_pub.publish(path_msg)

        self.get_logger().info(
            f'Published {len(data["points"])} path points'
        )

def main(args=None):
    rclpy.init(args=args)
    node = DFSPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
