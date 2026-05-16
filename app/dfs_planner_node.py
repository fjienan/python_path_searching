#!/usr/bin/env python3
"""
DFS路径规划节点 - 简化版

功能：
- 仅订阅 kfs_data 话题
- 将地图上每个位置的kfs值传递给DFSPlanner处理
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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

        self.GRID_ROWS = self.get_parameter('grid_rows').value
        self.GRID_COLS = self.get_parameter('grid_cols').value
        self.START_ROW = self.get_parameter('start_row').value
        self.START_COL = self.get_parameter('start_col').value

        # AStar中的参数注入与GridConverter初始化
        self.declare_parameter('map_origin', [3.2, 1.2, 0.0])
        self.declare_parameter('grid_resolution', 1.2)
        
        map_origin_raw = self.get_parameter('map_origin').value
        self.MAP_ORIGIN = tuple(map_origin_raw)
        self.GRID_RESOLUTION = self.get_parameter('grid_resolution').value

        self.grid_converter = GridConverter(
            grid_rows=self.GRID_ROWS,
            grid_cols=self.GRID_COLS,
            map_origin=self.MAP_ORIGIN,
            grid_resolution=self.GRID_RESOLUTION
        )

        # 初始化外部的DFS规划器
        self.planner = DFSPlanner(self.GRID_COLS, self.GRID_ROWS, logger=self.get_logger())
        self.path = None

        self.kfs_grid = np.zeros((self.GRID_ROWS, self.GRID_COLS), dtype=int)
        

        kfs_topic = self.declare_parameter('kfs_grid_data', '/kfs_grid_data').value
        path_topic = self.declare_parameter('planning_path', '/planning/path').value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.kfs_data_sub = self.create_subscription(
            String, kfs_topic, self.kfs_data_callback, qos_profile
        )
        
        # Path 发布器
        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(Path, path_topic, path_qos)

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
            self.get_logger().info(f'Received kfs_data:\n{grid}')
            
            # 将仅仅包含kfs值的地图网格扔给DFSPlanner
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
                    now_yaw = 0
                    if step.y == 0: now_yaw = 0
                    elif step.yaw ==1: now_yaw = math.pi/2
                    elif step.yaw ==2: now_yaw = -math.pi/2
                    self.get_logger().info(f'  Step {i+1}/{len(self.path[0])}: x={step.x}, y={step.y}, yaw={math.degrees(now_yaw)}, require_can_go={step.require_can_go}, send_can_do={step.send_can_do}')
                
                # 发布路径
                self.publish_path(self.path[0])

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing kfs_data: {e}')
    
    def delete_path(self):
        """删除旧路径（发布一个空的路径）"""
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty_path)

    def publish_path(self, steps):
        """发布路径（将grid索引转换为以此为起始原点的物理坐标）"""
        self.delete_path()

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, step in enumerate(steps):
            x, y = self.grid_converter.grid_to_map(step.x, step.y)

            # 方向转为弧度计算
            now_yaw = 0
            if step.y == 0: 
                now_yaw = 0
            elif step.yaw == 1: 
                now_yaw = math.pi/2
            elif step.yaw == 2: 
                now_yaw = -math.pi/2

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            # x y
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            # raw -> 四元数
            half_yaw = now_yaw * 0.5
            pose.pose.orientation.w = math.cos(half_yaw)
            pose.pose.orientation.z = math.sin(half_yaw)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published new path with {len(path_msg.poses)} poses')

def main(args=None):
    rclpy.init(args=args)
    node = DFSPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
