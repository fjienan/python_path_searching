#!/usr/bin/env python3
"""
A*路径规划节点

功能：
- 订阅odom_world、kfs_data和trigger话题
- 订阅/planning/direction话题用于方向选择
- 使用A*算法规划从当前位置到目标区域的路径
- 基于KFS网格判定障碍物和移动代价
- 发布规划路径到/planning/path
- 使用定时器每0.1s检查并规划路径

障碍物规则：
- kfs=3（假的kfs）: 障碍物，不可通过
- kfs=1（r1的kfs）: 默认视为障碍物，但可通过方向覆盖
- kfs=2（r2的kfs）: 可通过的奖励区域
- kfs=0（无kfs）: 可通过
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import json
import math
import numpy as np

from core.astar import AStarPlanner
from core.grid_utils import GridConverter


class AStarPlannerNode(Node):
    """A*路径规划节点"""

    def __init__(self):
        super().__init__('astar_planner')

        # 从参数服务器读取 grid 公共参数
        self.declare_parameter('grid.map_origin', [3.2, 1.2, 0.0])
        self.declare_parameter('grid.grid_rows', 4)
        self.declare_parameter('grid.grid_cols', 3)
        self.declare_parameter('grid.grid_resolution', 1.2)
        self.declare_parameter('grid.goal_grid_0', [3, 0])
        self.declare_parameter('grid.goal_grid_1', [3, 1])
        self.declare_parameter('grid.goal_grid_2', [3, 2])

        map_origin_raw = self.get_parameter('grid.map_origin').value
        self.MAP_ORIGIN = tuple(map_origin_raw)
        self.GRID_ROWS = self.get_parameter('grid.grid_rows').value
        self.GRID_COLS = self.get_parameter('grid.grid_cols').value
        self.GRID_RESOLUTION = self.get_parameter('grid.grid_resolution').value
        self.GOAL_GRIDS = [
            list(self.get_parameter('grid.goal_grid_0').value),
            list(self.get_parameter('grid.goal_grid_1').value),
            list(self.get_parameter('grid.goal_grid_2').value),
        ]

        self.planner = AStarPlanner(
            grid_rows=self.GRID_ROWS,
            grid_cols=self.GRID_COLS,
            grid_resolution=self.GRID_RESOLUTION,
            map_origin=self.MAP_ORIGIN
        )
        self.grid_converter = GridConverter(
            grid_rows=self.GRID_ROWS,
            grid_cols=self.GRID_COLS,
            map_origin=self.MAP_ORIGIN,
            grid_resolution=self.GRID_RESOLUTION
        )

        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.has_odom = False
        self.has_kfs_data = False
        self.trigger_received = True

        self.kfs_grid = np.zeros((self.GRID_ROWS, self.GRID_COLS), dtype=int)
        self.direction_override = None

        self.max_r2kfs_count = self.declare_parameter('max_r2kfs_count', 2).value
        self.allowed_r2kfs_positions = set()

        kfs_topic = self.declare_parameter('topics.kfs_grid_data', '/kfs_grid_data').value
        odom_topic = self.declare_parameter('topics.odom_world', '/odom_world').value
        trigger_topic = self.declare_parameter('topics.planning_trigger', '/task/trigger').value
        direction_topic = self.declare_parameter('topics.planning_direction', '/planning/direction').value
        path_topic = self.declare_parameter('topics.planning_path', '/planning/path').value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.kfs_data_sub = self.create_subscription(
            String, kfs_topic, self.kfs_data_callback, qos_profile
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )
        self.trigger_sub = self.create_subscription(
            String, trigger_topic, self.trigger_callback, 10
        )
        self.direction_sub = self.create_subscription(
            String, direction_topic, self.direction_callback, 10
        )

        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(Path, path_topic, path_qos)

        self.planning_timer = self.create_timer(0.1, self.check_and_plan)

        self.get_logger().info('A* Planner Node started')
        self.get_logger().info(f'Goals: {self.GOAL_GRIDS}')

    def odom_callback(self, msg):
        """处理里程计消息，更新当前位置"""
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z
        self.has_odom = True

    def kfs_data_callback(self, msg):
        """处理kfs_grid_data消息"""
        try:
            grid_data = json.loads(msg.data)
            grid = np.array(grid_data['grid'])

            if grid.shape != (self.GRID_ROWS, self.GRID_COLS):
                self.get_logger().warn(f'Invalid grid dimensions: {grid.shape}')
                return

            self.kfs_grid = grid
            self.has_kfs_data = True
            self.trigger_received = True
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
        """处理方向选择消息"""
        direction = msg.data.strip().lower()
        self.trigger_received = True
        if direction == "left":
            self.direction_override = "left"
            self.get_logger().info('Direction override: LEFT')
        elif direction == "right":
            self.direction_override = "right"
            self.get_logger().info('Direction override: RIGHT')
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
                self.delete_path()
            self.trigger_received = False

    def is_obstacle(self, row: int, col: int) -> bool:
        """检查grid坐标是否为障碍物"""
        if not self.grid_converter.is_in_grid(row, col):
            return True

        kfs_value = self.kfs_grid[row, col]

        if kfs_value == 1:
            if self.direction_override == "left" and col == 2:
                return False
            elif self.direction_override == "right" and col == 0:
                return False

        if kfs_value == 2 and self.allowed_r2kfs_positions:
            if (row, col) not in self.allowed_r2kfs_positions:
                return True

        return kfs_value == 3 or kfs_value == 1

    def get_cost(self, row: int, col: int) -> float:
        """获取移动到该格子的代价"""
        if self.is_obstacle(row, col):
            return float('inf')

        kfs_value = self.kfs_grid[row, col]
        if kfs_value == 2 and (row, col) in self.allowed_r2kfs_positions:
            return 0.0
        return 1.0

    def plan_path_to_goal(self, start_row, start_col, goal_row, goal_col):
        """规划到单个目标点的路径"""
        return self.planner.plan_path(
            (start_row, start_col),
            (goal_row, goal_col),
            is_obstacle_fn=self.is_obstacle,
            get_cost_fn=self.get_cost
        )

    def plan_path(self):
        """
        执行A*路径规划到多个目标点，选择最优路径
        确保选择最优的max_r2kfs_count个r2kfs
        """
        add_new_start = False
        start_row, start_col = self.grid_converter.map_to_grid(
            self.current_pos[0], self.current_pos[1]
        )
        entry_r2kfs_col = None

        if start_row == -1 and start_col == 1:
            add_new_start = True
            start_row, start_col = 0, 1
        elif start_row == -1:
            r2kfs_col = self.get_r2kfs_col_in_first_row()
            if r2kfs_col != -1:
                add_new_start = True
                entry_r2kfs_col = r2kfs_col
                start_row, start_col = 0, r2kfs_col
                self.get_logger().info(f"Starting at (-1, {start_col}), entering at (0, {r2kfs_col})")
            elif start_col == 0:
                add_new_start = True
                start_row, start_col = 0, 0

        self.get_logger().info(f'Planning from [{start_row}, {start_col}] to multiple goals')
        self.get_logger().info(f'Max allowed r2kfs count: {self.max_r2kfs_count}')

        all_r2kfs_positions = self.planner.find_all_positions(self.kfs_grid, 2)
        self.get_logger().info(f'All r2kfs positions: {all_r2kfs_positions}')

        self.allowed_r2kfs_positions = set()
        best_path = None
        best_cost = float('inf')
        best_goal = None
        best_path_r2kfs = set()

        for goal_row, goal_col in self.GOAL_GRIDS:
            path, cost = self.plan_path_to_goal(start_row, start_col, goal_row, goal_col)
            if path is not None:
                path_r2kfs = self.planner.count_cells_around_path(path, self.kfs_grid, 2)
                self.get_logger().info(f'Path to [{goal_row},{goal_col}]: Cost {cost:.2f}, r2kfs count {len(path_r2kfs)}')

                if len(path_r2kfs) >= self.max_r2kfs_count:
                    if cost < best_cost:
                        best_path = path
                        best_cost = cost
                        best_goal = [goal_row, goal_col]
                        best_path_r2kfs = path_r2kfs
                elif not best_path:
                    best_path = path
                    best_cost = cost
                    best_goal = [goal_row, goal_col]
                    best_path_r2kfs = path_r2kfs

        if not best_path:
            self.get_logger().warn('No path found to any goal!')
            return None

        if len(best_path_r2kfs) < self.max_r2kfs_count:
            self.get_logger().info('Path does not satisfy r2kfs count requirement, finding a new path...')

            import itertools
            all_r2kfs = self.planner.find_all_positions(self.kfs_grid, 2)
            missing_r2kfs = [p for p in all_r2kfs if p not in best_path_r2kfs]

            temp_best_path = None
            temp_best_cost = float('inf')
            temp_best_r2kfs = set()

            for goal_row, goal_col in self.GOAL_GRIDS:
                for k in range(1, min(self.max_r2kfs_count - len(best_path_r2kfs), len(missing_r2kfs)) + 1):
                    for additional in itertools.combinations(missing_r2kfs, k):
                        required_positions = best_path_r2kfs.union(set(additional))
                        self.allowed_r2kfs_positions = required_positions

                        path, cost = self.plan_path_to_goal(start_row, start_col, goal_row, goal_col)
                        if path is not None:
                            path_r2kfs = self.planner.count_cells_around_path(path, self.kfs_grid, 2)

                            if len(path_r2kfs) >= self.max_r2kfs_count and cost < temp_best_cost:
                                temp_best_path = path
                                temp_best_cost = cost
                                temp_best_r2kfs = path_r2kfs

            if temp_best_path:
                self.get_logger().info(f'Found path with {len(temp_best_r2kfs)} r2kfs')
                best_path = temp_best_path
                best_cost = temp_best_cost
                best_path_r2kfs = temp_best_r2kfs
            else:
                self.get_logger().info(f'No path with sufficient r2kfs found, using best found with {len(best_path_r2kfs)}')

        self.get_logger().info(f'Selected path to {best_goal} with cost={best_cost:.2f}')

        if add_new_start:
            if entry_r2kfs_col is not None:
                full_path = [(-1, entry_r2kfs_col), (0, entry_r2kfs_col)]
                for point in best_path:
                    if point != (0, entry_r2kfs_col):
                        full_path.append(point)
                return full_path
            elif start_col == 0:
                return [(-1, 0)] + best_path
            else:
                return [(-1, 1)] + best_path
        else:
            return best_path

    def get_r2kfs_col_in_first_row(self) -> int:
        """检测第一行是否存在r2kfs，返回列索引，未找到返回-1"""
        if self.has_kfs_data:
            for col in range(self.GRID_COLS):
                if self.kfs_grid[0, col] == 2:
                    return col
        return -1

    def delete_path(self):
        """删除旧路径（发布一个空的路径）"""
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty_path)

    def publish_path(self, path):
        """发布路径（将grid索引转换为map坐标，每个点附带朝向角）"""
        self.delete_path()

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, (row, col) in enumerate(path):
            x, y = self.grid_converter.grid_to_map(row, col)

            if i > 0:
                prev_row, prev_col = path[i - 1]
                prev_x, prev_y = self.grid_converter.grid_to_map(prev_row, prev_col)
                target_yaw = math.atan2(y - prev_y, x - prev_x)
            else:
                target_yaw = 0.0

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            half_yaw = target_yaw * 0.5
            pose.pose.orientation.w = math.cos(half_yaw)
            pose.pose.orientation.z = math.sin(half_yaw)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0

            path_msg.poses.append(pose)

        # 再加一个点，往下走一格，使机器人完全离开场地
        if path:
            last_row, last_col = path[-1]
            exit_row = last_row + 1
            exit_x, exit_y = self.grid_converter.grid_to_map(exit_row, last_col)
            exit_pose = PoseStamped()
            exit_pose.header.frame_id = 'map'
            exit_pose.header.stamp = path_msg.header.stamp
            exit_pose.pose.position.x = float(exit_x)
            exit_pose.pose.position.y = float(exit_y)
            exit_pose.pose.position.z = 0.0
            last_x, last_y = self.grid_converter.grid_to_map(last_row, last_col)
            exit_yaw = math.atan2(exit_y - last_y, exit_x - last_x)
            half_exit_yaw = exit_yaw * 0.5
            exit_pose.pose.orientation.w = math.cos(half_exit_yaw)
            exit_pose.pose.orientation.z = math.sin(half_exit_yaw)
            exit_pose.pose.orientation.x = 0.0
            exit_pose.pose.orientation.y = 0.0
            path_msg.poses.append(exit_pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published new path with {len(path_msg.poses)} poses')


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
