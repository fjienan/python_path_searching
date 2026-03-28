#!/usr/bin/env python3
"""
A*路径规划节点

功能：
- 订阅odom_world、kfs_data和trigger话题
- 订阅/planning/direction话题用于方向选择
- 使用A*算法规划从当前位置（odom_world）到[3][1]的路径
- 基于KFS网格判定障碍物和移动代价
- 发布规划路径到/planning/path
- 使用定时器每0.1s检查并规划路径

障碍物规则：
- kfs=3（假的kfs）: 障碍物，不可通过
- kfs=1（r1的kfs）: 默认视为障碍物，但可通过方向覆盖：
  * 如果direction="left": col==2位置的kfs=1不算障碍物
  * 如果direction="right": col==0位置的kfs=1不算障碍物
- kfs=2（r2的kfs）: cost=2
- kfs=0（无kfs）: cost=1
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import json
import numpy as np
import heapq
from math import floor


class AStarPlannerNode(Node):
    """A*路径规划节点"""
    
    def __init__(self):
        super().__init__('astar_planner')
        
        # 网格配置
        self.grid_rows = 4
        self.grid_cols = 3
        
        # Map2配置（用于坐标转换）
        self.map2_origin = [3.2, 1.2, 0.0]  # [x, y, theta]
        self.grid_resolution = 1.2  # 米/单元格
        
        # 多个目标点（grid索引，row和col），按优先级排序
        self.goal_grids = [[3, 0], [3, 1], [3, 2]]  # 优先级：[3,0] > [3,1] > [3,2]
        
        # 当前机器人位置（从odom获取）
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.has_odom = False
        
        # 数据标志
        self.has_kfs_data = False
        self.trigger_received = True
        
        # 存储KFS网格数据
        self.kfs_grid = np.zeros((self.grid_rows, self.grid_cols), dtype=int)
        
        # 方向覆盖：None, "left", "right"
        self.direction_override = None
        
        # 配置QoS（使用TRANSIENT_LOCAL以接收初始消息）
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 创建订阅者
        self.kfs_data_sub = self.create_subscription(
            String,
            '/kfs_grid_data',
            self.kfs_data_callback,
            qos_profile
        )
        
        # 订阅odom_world以获取当前位置
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_world',
            self.odom_callback,
            10
        )
        
        self.trigger_sub = self.create_subscription(
            String,
            '/task/trigger',
            self.trigger_callback,
            10
        )
        
        # 方向选择订阅（用于障碍物覆盖）
        self.direction_sub = self.create_subscription(
            String,
            '/planning/direction',
            self.direction_callback,
            10
        )
        
        # 创建发布者（使用TRANSIENT_LOCAL QoS）
        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(
            Path,
            '/planning/path',
            path_qos
        )
        
        # 创建定时器，每0.1s检查并规划路径
        self.planning_timer = self.create_timer(0.1, self.check_and_plan)
        
        self.get_logger().info('A* Planner Node started')
        self.get_logger().info(f'Goals: {self.goal_grids} (start from current odom position)')
    
    def odom_callback(self, msg):
        """处理里程计消息，更新当前位置"""
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z
        self.has_odom = True
    
    def map_to_grid_coords(self, x, y):
        """
        将map坐标转换为grid索引
        
        Args:
            x, y: map坐标
            
        Returns:
            tuple: (row, col) grid索引
        """
        row = floor((x - self.map2_origin[0]) / self.grid_resolution)
        col = floor((y - self.map2_origin[1]) / self.grid_resolution)
        return row, col
    
    def kfs_data_callback(self, msg):
        """处理kfs_grid_data消息"""
        try:
            grid_data = json.loads(msg.data)
            grid = np.array(grid_data['grid'])
            
            if grid.shape != (self.grid_rows, self.grid_cols):
                self.get_logger().warn(f'Invalid grid dimensions: {grid.shape}')
                return
            
            self.kfs_grid = grid
            self.has_kfs_data = True
            self.trigger_received = True  # 每次收到kfs_data都将trigger设为true
            
            self.get_logger().info(f'Received kfs_data:\n{grid}')
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing kfs_data: {e}')
    
    def trigger_callback(self, msg):
        """处理trigger消息"""
        self.trigger_received = True
        self.get_logger().info('Received trigger signal')
    
    def direction_callback(self, msg):
        """处理方向选择消息，仅更新direction_override，不触发规划"""
        direction = msg.data.strip().lower()
        self.trigger_received=True
        if direction == "left":
            self.direction_override = "left"
            self.get_logger().info('Direction override: LEFT (col==2 kfs=1 not obstacle)')
        elif direction == "right":
            self.direction_override = "right"
            self.get_logger().info('Direction override: RIGHT (col==0 kfs=1 not obstacle)')
        else:
            self.get_logger().warn(f'Unknown direction: {direction}')
    
    def check_and_plan(self):
        """定时器回调：检查是否所有数据都准备好，如果是则执行规划"""
        if self.has_kfs_data and self.has_odom and self.trigger_received:
            self.get_logger().info('All data ready, starting A* planning...')
            path = self.plan_path()
            if path is not None:
                self.publish_path(path)
            else:
                self.get_logger().warn('Path planning failed')
                # 规划失败时也删除旧路径
                self.delete_path()
            # 重置trigger标志，避免重复规划
            self.trigger_received = False
    
    def is_obstacle(self, row, col):
        """
        检查grid坐标是否为障碍物
        
        Args:
            row, col: grid坐标
            
        Returns:
            bool: True表示障碍物，False表示可通过
        """
        if row < 0 or row >= self.grid_rows or col < 0 or col >= self.grid_cols:
            return True
        
        kfs_value = self.kfs_grid[row, col]
        
        # 方向覆盖逻辑
        if kfs_value == 1:  # r1的kfs
            if self.direction_override == "left" and col == 2:
                return False  # 左侧：col==2的kfs=1不算障碍物
            elif self.direction_override == "right" and col == 0:
                return False  # 右侧：col==0的kfs=1不算障碍物
        
        # kfs=3（假的kfs）或kfs=1（r1的kfs，无覆盖时）视为障碍物
        return kfs_value == 3 or kfs_value == 1
    
    def is_adjacent_to_r2kfs(self, row, col):
        """
        检查格子是否与r2kfs(kfs=2)相邻

        Args:
            row, col: grid坐标

        Returns:
            bool: True表示与r2kfs相邻
        """
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 上下左右
            adj_row = row + dr
            adj_col = col + dc
            if 0 <= adj_row < self.grid_rows and 0 <= adj_col < self.grid_cols:
                if self.kfs_grid[adj_row, adj_col] == 2:
                    return True
        return False

    def get_r2kfs_col_in_first_row(self):
        """
        检测第一行（row=0）是否存在 r2kfs（kfs_value=2），返回第一个找到的列索引

        Returns:
            int: 找到的 r2kfs 列索引，未找到返回 -1
        """
        if self.has_kfs_data:
            # 遍历第一行的所有列
            for col in range(self.grid_cols):
                if self.kfs_grid[0, col] == 2:
                    return col
        return -1

    def get_cost(self, row, col):
        """
        获取移动到该格子的代价

        Args:
            row, col: grid坐标

        Returns:
            float: 移动代价（障碍物返回inf）
        """
        if self.is_obstacle(row, col):
            return float('inf')

        kfs_value = self.kfs_grid[row, col]
        if kfs_value == 2:  # r2的kfs
            return 0.0  # r2kfs权重为0，最佳
        else:  # kfs=0，无kfs
            return 1.0
    
    def heuristic(self, row1, col1, row2, col2):
        """
        启发式函数：曼哈顿距离（绝对值边长相加）
        
        Args:
            row1, col1: 起点坐标
            row2, col2: 终点坐标
            
        Returns:
            float: 曼哈顿距离
        """
        return abs(row1 - row2) + abs(col1 - col2)
    
    def get_neighbors(self, row, col):
        """
        获取四联通邻居（上下左右）
        
        Args:
            row, col: 当前坐标
            
        Returns:
            list: [(row, col), ...] 邻居坐标列表
        """
        neighbors = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 上下左右
            new_row = row + dr
            new_col = col + dc
            if 0 <= new_row < self.grid_rows and 0 <= new_col < self.grid_cols:
                neighbors.append((new_row, new_col))
        return neighbors
    
    def plan_path_to_goal(self, start_row, start_col, goal_row, goal_col):
        """
        规划到单个目标点的路径
        
        Args:
            start_row, start_col: 起点grid坐标
            goal_row, goal_col: 终点grid坐标
            
        Returns:
            tuple: (path, cost) 路径点列表和总代价，失败返回 (None, float('inf'))
        """
        # 检查起点和终点
        if self.is_obstacle(start_row, start_col):
            return None, float('inf')
        
        if self.is_obstacle(goal_row, goal_col):
            return None, float('inf')
        
        # A*算法
        open_set = []  # (f_score, row, col)
        closed_set = set()  # 已处理的节点集合
        came_from = {}  # (row, col) -> (row, col)
        g_score = {}  # (row, col) -> float
        
        start_key = (start_row, start_col)
        goal_key = (goal_row, goal_col)
        
        g_score[start_key] = 0.0
        f_score = self.heuristic(start_row, start_col, goal_row, goal_col)
        
        heapq.heappush(open_set, (f_score, start_row, start_col))
        
        while open_set:
            current_f, current_row, current_col = heapq.heappop(open_set)
            current_key = (current_row, current_col)
            
            # 检查是否已经在closed_set中（避免重复处理）
            if current_key in closed_set:
                continue
            
            # 将当前节点加入closed_set
            closed_set.add(current_key)
            
            # 检查是否到达目标
            if current_row == goal_row and current_col == goal_col:
                # 重构路径
                path = []
                while current_key in came_from:
                    path.append((current_key[0], current_key[1]))
                    current_key = came_from[current_key]
                path.append((start_row, start_col))
                path.reverse()
                path.append((goal_row+1, goal_col))
                
                # 计算路径总代价
                total_cost = g_score[goal_key]
                
                return path, total_cost
            
            # 检查邻居
            for neighbor_row, neighbor_col in self.get_neighbors(current_row, current_col):
                neighbor_key = (neighbor_row, neighbor_col)
                
                # 跳过已处理的节点
                if neighbor_key in closed_set:
                    continue
                
                # 获取移动代价
                move_cost = self.get_cost(neighbor_row, neighbor_col)
                if move_cost == float('inf'):
                    continue  # 跳过障碍物
                
                tentative_g = g_score.get(current_key, float('inf')) + move_cost
                
                # 如果找到更短的路径，更新
                if neighbor_key not in g_score or tentative_g < g_score[neighbor_key]:
                    came_from[neighbor_key] = current_key
                    g_score[neighbor_key] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor_row, neighbor_col, goal_row, goal_col)
                    heapq.heappush(open_set, (f_score, neighbor_row, neighbor_col))
        
        # 未找到路径
        return None, float('inf')
    
    def plan_path(self):
        """
        执行A*路径规划到多个目标点，选择最优路径
        
        Returns:
            list: [(row, col), ...] 路径点列表，失败返回None
        """
        # 从当前位置计算起点grid坐标
        add_new_start=False
        start_row, start_col = self.map_to_grid_coords(self.current_pos[0], self.current_pos[1])
        if start_row==-1 and start_col==1:
            add_new_start=True
            start_row, start_col = 0,1
        elif start_row==-1:  # 如果在 (-1, *) 其他位置
            # 检测是否有 r2kfs
            r2kfs_col = self.get_r2kfs_col_in_first_row()
            if r2kfs_col != -1:
                add_new_start=True
                start_row, start_col = 0, r2kfs_col
                self.get_logger().info(f"Starting at (-1, {start_col}) and entering grid at (0, {r2kfs_col}) due to r2kfs")
            elif start_col == 0:  # 如果在 (-1, 0) 且无 r2kfs
                add_new_start=True
                start_row, start_col = 0,0
                self.get_logger().info("Starting at (-1, 0) and entering grid at (0, 0) (no r2kfs in first row)")

        self.get_logger().info(f'Planning from current position: grid [{start_row}, {start_col}],currentpose[{self.current_pos[0],self.current_pos[1]}] to multiple goals')
        
        # 规划到所有目标点，选择cost最小的
        best_path = None
        best_cost = float('inf')
        best_goal = None
        
        for goal_row, goal_col in self.goal_grids:
            path, cost = self.plan_path_to_goal(start_row, start_col, goal_row, goal_col)
            if path is not None:
                self.get_logger().info(f'Path to [{goal_row}, {goal_col}]: cost={cost:.2f}, length={len(path)}')
                # 选择cost更小的（cost相等时，列表前面的目标点会被优先选择）
                if cost < best_cost:
                    best_path = path
                    best_cost = cost
                    best_goal = [goal_row, goal_col]
            else:
                self.get_logger().warn(f'No path found to [{goal_row}, {goal_col}]')
        
        if best_path is None:
            self.get_logger().warn('No path found to any goal!')
            return None

        self.get_logger().info(f'Selected path to {best_goal} with cost={best_cost:.2f}, length={len(best_path)}')

        r2kfs_col = self.get_r2kfs_col_in_first_row()
        if add_new_start:
            if r2kfs_col != -1:
                # 直接构造路径：先从 (-1, col) 到 (0, col)，然后再继续原路径
                full_path = [(-1, r2kfs_col), (0, r2kfs_col)]
                # 添加后续路径，但跳过可能重复的 (0, r2kfs_col)
                for point in best_path:
                    if point != (0, r2kfs_col):
                        full_path.append(point)
                return full_path
            elif start_col == 0:
                return [(-1, 0)] + best_path
            else:
                return [(-1, 1)] + best_path
        else:
            return best_path
    
    def grid_to_map_coords(self, row, col):
        """
        将grid索引转换为map坐标
        
        Args:
            row, col: grid索引
            
        Returns:
            tuple: (x, y) map坐标
        """
        x = self.map2_origin[0] + row * self.grid_resolution + 0.5 * self.grid_resolution
        y = self.map2_origin[1] + col * self.grid_resolution + 0.5 * self.grid_resolution
        return x, y
    
    def delete_path(self):
        """删除旧路径（发布一个空的路径）"""
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty_path)
        self.get_logger().info('Deleted old path (published empty path)')
    
    def publish_path(self, path):
        """
        发布路径（将grid索引转换为map坐标）
        先删除旧路径，再发布新路径
        
        Args:
            path: [(row, col), ...] 路径点列表（grid索引）
        """
        # 先删除旧路径
        self.delete_path()
        
        # 发布新路径
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for row, col in path:
            # 将grid索引转换为map坐标
            x, y = self.grid_to_map_coords(row, col)
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published new path with {len(path_msg.poses)} poses')


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
