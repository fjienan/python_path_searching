"""
网格坐标转换工具

提供网格坐标与物理坐标之间的转换功能
"""

from math import floor
from typing import Tuple


class GridConverter:
    """
    网格坐标转换器

    用于将物理坐标(map坐标)转换为网格索引，以及反向转换

    Example:
        >>> converter = GridConverter(
        ...     grid_rows=4,
        ...     grid_cols=3,
        ...     map_origin=(3.2, 1.2, 0.0),
        ...     grid_resolution=1.2
        ... )
        >>> row, col = converter.map_to_grid(4.7, 1.8)
        >>> x, y = converter.grid_to_map(1, 1)
    """

    def __init__(
        self,
        grid_rows: int,
        grid_cols: int,
        map_origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        grid_resolution: float = 1.0
    ):
        """
        Args:
            grid_rows: 网格行数
            grid_cols: 网格列数
            map_origin: 地图原点坐标 [x, y, theta]
            grid_resolution: 网格分辨率（米/单元格）
        """
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        self.map_origin = map_origin
        self.grid_resolution = grid_resolution

    def map_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        将map坐标转换为grid索引

        Args:
            x, y: map坐标

        Returns:
            tuple: (row, col) grid索引
        """
        row = floor((x - self.map_origin[0]) / self.grid_resolution)
        col = floor((y - self.map_origin[1]) / self.grid_resolution)
        return row, col

    def grid_to_map(self, row: int, col: int) -> Tuple[float, float]:
        """
        将grid索引转换为map坐标（格子中心点）

        Args:
            row, col: grid索引

        Returns:
            tuple: (x, y) map坐标
        """
        x = self.map_origin[0] + row * self.grid_resolution + 0.5 * self.grid_resolution
        y = self.map_origin[1] + col * self.grid_resolution + 0.5 * self.grid_resolution
        return x, y

    def is_in_grid(self, row: int, col: int) -> bool:
        """检查grid坐标是否在有效范围内"""
        return 0 <= row < self.grid_rows and 0 <= col < self.grid_cols

    def get_grid_center(self, row: int, col: int) -> Tuple[float, float]:
        """
        获取指定格子中心的map坐标

        Args:
            row, col: grid索引

        Returns:
            tuple: (x, y) 格子中心的map坐标
        """
        return self.grid_to_map(row, col)

    def get_neighbors(
        self,
        row: int,
        col: int,
        include_diagonal: bool = False
    ) -> list:
        """
        获取指定格子的邻居格子

        Args:
            row, col: 当前格子坐标
            include_diagonal: 是否包含对角邻居

        Returns:
            list: 邻居格子坐标列表
        """
        neighbors = []

        if include_diagonal:
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = row + dr, col + dc
                    if self.is_in_grid(nr, nc):
                        neighbors.append((nr, nc))
        else:
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = row + dr, col + dc
                if self.is_in_grid(nr, nc):
                    neighbors.append((nr, nc))

        return neighbors
