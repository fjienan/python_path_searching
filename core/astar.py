"""
A*路径规划算法核心实现

基于网格的A*算法，支持：
- 曼哈顿距离启发式
- 四联通邻居
- 可配置的障碍物判定和移动代价
"""

import heapq
from typing import List, Tuple, Set, Callable, Optional, Dict


class AStarPlanner:
    """
    A*路径规划器核心类

    Attributes:
        grid_rows: 网格行数
        grid_cols: 网格列数
        grid_resolution: 网格分辨率（米/单元格）
        map_origin: 地图原点坐标 [x, y, theta]
    """

    def __init__(
        self,
        grid_rows: int,
        grid_cols: int,
        grid_resolution: float = 1.0,
        map_origin: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    ):
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        self.grid_resolution = grid_resolution
        self.map_origin = map_origin

    def is_in_bounds(self, row: int, col: int) -> bool:
        """检查坐标是否在网格范围内"""
        return 0 <= row < self.grid_rows and 0 <= col < self.grid_cols

    def heuristic(self, row1: int, col1: int, row2: int, col2: int) -> float:
        """
        启发式函数：曼哈顿距离

        Args:
            row1, col1: 起点坐标
            row2, col2: 终点坐标

        Returns:
            float: 曼哈顿距离
        """
        return abs(row1 - row2) + abs(col1 - col2)

    def get_neighbors(self, row: int, col: int) -> List[Tuple[int, int]]:
        """
        获取四联通邻居（上下左右）

        Args:
            row, col: 当前坐标

        Returns:
            list: 邻居坐标列表
        """
        neighbors = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_row, new_col = row + dr, col + dc
            if self.is_in_bounds(new_row, new_col):
                neighbors.append((new_row, new_col))
        return neighbors

    def get_neighbors_diagonal(self, row: int, col: int) -> List[Tuple[int, int]]:
        """
        获取八联通邻居（包含对角）

        Args:
            row, col: 当前坐标

        Returns:
            list: 邻居坐标列表
        """
        neighbors = []
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                new_row, new_col = row + dr, col + dc
                if self.is_in_bounds(new_row, new_col):
                    neighbors.append((new_row, new_col))
        return neighbors

    def plan_path(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        is_obstacle_fn: Callable[[int, int], bool],
        get_cost_fn: Callable[[int, int], float] = None
    ) -> Tuple[Optional[List[Tuple[int, int]]], float]:
        """
        规划从起点到终点的路径

        Args:
            start: 起点坐标 (row, col)
            goal: 终点坐标 (row, col)
            is_obstacle_fn: 障碍物判定函数，接收(row, col)，返回True表示障碍物
            get_cost_fn: 移动代价函数，接收(row, col)，返回移动代价（默认返回1.0）

        Returns:
            tuple: (path, cost) 路径点列表和总代价，失败返回(None, inf)
        """
        start_row, start_col = start
        goal_row, goal_col = goal

        if is_obstacle_fn(start_row, start_col):
            return None, float('inf')

        if is_obstacle_fn(goal_row, goal_col):
            return None, float('inf')

        if get_cost_fn is None:
            def default_cost(r, c):
                return 1.0
            get_cost_fn = default_cost

        open_set = []
        closed_set = set()
        came_from = {}
        g_score = {}

        start_key = (start_row, start_col)
        goal_key = (goal_row, goal_col)

        g_score[start_key] = 0.0
        f_score = self.heuristic(start_row, start_col, goal_row, goal_col)
        heapq.heappush(open_set, (f_score, start_row, start_col))

        while open_set:
            _, current_row, current_col = heapq.heappop(open_set)
            current_key = (current_row, current_col)

            if current_key in closed_set:
                continue

            closed_set.add(current_key)

            if current_row == goal_row and current_col == goal_col:
                path = []
                while current_key in came_from:
                    path.append((current_key[0], current_key[1]))
                    current_key = came_from[current_key]
                path.append((start_row, start_col))
                path.reverse()
                return path, g_score[goal_key]

            for neighbor_row, neighbor_col in self.get_neighbors(current_row, current_col):
                neighbor_key = (neighbor_row, neighbor_col)

                if neighbor_key in closed_set:
                    continue

                if is_obstacle_fn(neighbor_row, neighbor_col):
                    continue

                move_cost = get_cost_fn(neighbor_row, neighbor_col)
                tentative_g = g_score.get(current_key, float('inf')) + move_cost

                if neighbor_key not in g_score or tentative_g < g_score[neighbor_key]:
                    came_from[neighbor_key] = current_key
                    g_score[neighbor_key] = tentative_g
                    f_score = tentative_g + self.heuristic(
                        neighbor_row, neighbor_col, goal_row, goal_col
                    )
                    heapq.heappush(open_set, (f_score, neighbor_row, neighbor_col))

        return None, float('inf')

    def find_all_positions(
        self,
        grid,
        target_value: int
    ) -> List[Tuple[int, int]]:
        """
        找到网格中所有指定值的格子位置

        Args:
            grid: 网格数据（二维numpy数组或列表）
            target_value: 目标值

        Returns:
            list: 满足条件的坐标列表 [(row, col), ...]
        """
        positions = []
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                try:
                    if grid[row, col] == target_value:
                        positions.append((row, col))
                except (TypeError, IndexError):
                    if grid[row][col] == target_value:
                        positions.append((row, col))
        return positions

    def count_cells_around_path(
        self,
        path: List[Tuple[int, int]],
        grid,
        target_value: int
    ) -> Set[Tuple[int, int]]:
        """
        计算路径上和周围四格内的所有指定值的格子（去重）

        Args:
            path: 路径点列表 [(row, col), ...]
            grid: 网格数据
            target_value: 目标值

        Returns:
            set: 满足条件的坐标集合
        """
        positions = set()

        for row, col in path:
            if not self.is_in_bounds(row, col):
                continue

            if grid[row, col] == target_value:
                positions.add((row, col))

            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = row + dr, col + dc
                if self.is_in_bounds(nr, nc):
                    if grid[nr, nc] == target_value:
                        positions.add((nr, nc))

        return positions

    def select_optimal_positions(
        self,
        positions: List[Tuple[int, int]],
        start: Tuple[int, int],
        goal: Tuple[int, int],
        max_count: int
    ) -> List[Tuple[int, int]]:
        """
        从位置列表中选择最优的位置组合

        选择距离起点和终点总距离最短、且大致在起点到终点路径上的位置

        Args:
            positions: 可选位置列表
            start: 起点坐标
            goal: 终点坐标
            max_count: 最大选择数量

        Returns:
            list: 选择的位置列表
        """
        if len(positions) <= max_count:
            return positions

        import itertools

        scored_positions = []
        start_row, start_col = start
        goal_row, goal_col = goal

        for row, col in positions:
            dist_to_start = abs(row - start_row) + abs(col - start_col)
            dist_to_goal = abs(row - goal_row) + abs(col - goal_col)
            score = -(dist_to_start + dist_to_goal)

            if (start_row <= row <= goal_row or goal_row <= row <= start_row) and \
               (start_col <= col <= goal_col or goal_col <= col <= start_col):
                score += 100

            scored_positions.append((score, row, col))

        scored_positions.sort(reverse=True, key=lambda x: x[0])
        candidates = [(row, col) for _, row, col in scored_positions]

        best_combination = None
        best_score = float('-inf')

        for combo in itertools.combinations(candidates, max_count):
            path_score = 0
            prev_pos = start

            for pos in combo:
                dist = abs(pos[0] - prev_pos[0]) + abs(pos[1] - prev_pos[1])
                path_score -= dist
                prev_pos = pos

            path_score -= abs(goal_row - prev_pos[0]) + abs(goal_col - prev_pos[1])

            in_path_count = 0
            for r, c in combo:
                if (start_row <= r <= goal_row or goal_row <= r <= start_row) and \
                   (start_col <= c <= goal_col or goal_col <= c <= start_col):
                    in_path_count += 1
                    path_score += 50

            if path_score > best_score:
                best_score = path_score
                best_combination = list(combo)

        if best_combination is None:
            best_combination = candidates[:max_count]

        return best_combination
