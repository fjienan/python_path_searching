"""
DFS路径规划算法核心实现

逻辑：使用DFS遍历每一条路径。

可行性：
1. 首排有kfs2一定要取，不去取的路径不考虑。
2. 第一梯队：kfs2在必经之路上的一定要取
   第二梯队：去掉第一梯队的kfs2，到达某位置时，朝向的地方刚好有的kfs2
   第三梯队：去掉第一、二梯队的kfs2，到达某位置时，通过转向能拿到的kfs2
3. 必经之路上穿过了所有4个kfs2的路径不考虑。(默认R2不具备捡起kfs2扔到一边的功能，捡起了一定拿下，而R2上放不下4个kfs2)
4. 拿到小于2个kfs2的路径不考虑。

路径评估：

第一优先：路径的代价=移动代价+转向代价+精细调整+捡起kfs2代价
第二优先：受影响的kfs1数量（越少越好）
第三优先：取到kfs2的数量 2个优先于3个


"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import json
import numpy as np
from core.step import Step

class DFSPlanner:
    def __init__(self, grid_cols, grid_rows, logger=None):
        self.logger = logger
        self.GRID_ROWS = grid_rows
        self.GRID_COLS = grid_cols
        self.DIRCTIONS = [(1, 0, 0), (0, 1, 1), (0, -1, 2)] 
        self.GRID = np.zeros((self.GRID_ROWS+2, self.GRID_COLS), dtype=int)
        self.visited = np.zeros((self.GRID_ROWS+2, self.GRID_COLS), dtype=bool)
        self.path = [[],[],[],[]]
        self.cnt_path = []
        self.eval_path=[]
        
        self.MOVE_COST = 6
        self.TURN_COST = 2
        self.ADJACENT_COST = 1
        self.FETCH_KFS2_COST = 6
    
    def plan_path(self, grid, stx, sty):
        """DFS路径规划算法实现"""
        self.GRID = np.vstack(([0, 0, 0], grid, [0, 0, 0]))
        # self.logger.info(f"GRID:\n{self.GRID}")
        self._dfs(stx+1, sty, 0)
        if len(self.path[2])==0 and len(self.path[3])==0:
            return None
        elif len(self.path[2])==0:
            return self.path[3][0]
        elif len(self.path[3])==0:
            return self.path[2][0]
        else:
            if self.path[2][0][1] < self.path[3][0][1]:
                return self.path[2][0]
            elif self.path[2][0][1] > self.path[3][0][1]:
                return self.path[3][0]
            else:
                if len(self.path[2][0][3]) < len(self.path[3][0][3]):
                    return self.path[2][0]
                elif len(self.path[2][0][3]) > len(self.path[3][0][3]):
                    return self.path[3][0]
                else:
                    return self.path[2][0]

    def _dfs(self, posX, posY, yaw):
        
        # self.logger.info(f'Visiting ({posX}, {posY}), yaw={yaw}')
        if posX==5:
            self._copy_path(self.cnt_path, self.eval_path)
            self._evaluate_path()
            return
        self.visited[posX][posY] = True
        self.cnt_path.append(Step(posX, posY, yaw))
        # self.logger.info(f"{self.visited}")
        for direction in self.DIRCTIONS:
            newX = posX + direction[0]
            newY = posY + direction[1]
            if newX<0 or newX> 5 or newY<0 or newY> 2:
                continue
            if self.visited[newX][newY] or self.GRID[newX][newY] == 3:
                continue
            if posX == 0 and newX == 1 and (2 in self.GRID[1]) and self.GRID[1][newY]!=2:
                continue
            # self.logger.info(f'At ({posX}, {posY}), trying to move to ({newX}, {newY}) with direction {direction[2]}')
           
            self._dfs(newX, newY, direction[2])
        self.visited[posX][posY] = False
        self.cnt_path.pop()
    
    def _copy_path(self, src, dst):
        dst.clear()
        for step in src:
            dst.append(Step(step.x, step.y, step.yaw, step.require_can_go, step.send_can_do))
    
    def _evaluate_path(self):
        grid_cnt = self.GRID.copy()
        #self.logger.info(f'{grid_cnt}')
        kfs2_on_the_way = []
        #for step in self.eval_path:
            #self.logger.info(f'step: x={step.x}, y={step.y}, yaw={step.yaw}, require_can_go={step.require_can_go}, send_can_do={step.send_can_do}')
        for step in self.eval_path:
            if grid_cnt[step.x][step.y] == 2:
                kfs2_on_the_way.append([step.x, step.y])
                grid_cnt[step.x][step.y] = 0
                
        if len(kfs2_on_the_way) == 4: return
        
        kfs2_by_the_way = []
        for step in self.eval_path:
            if 0<= step.x+self.DIRCTIONS[step.yaw][0] <= 5 and 0<= step.y+self.DIRCTIONS[step.yaw][1] <= 2:
                if grid_cnt[step.x+self.DIRCTIONS[step.yaw][0]][step.y+self.DIRCTIONS[step.yaw][1]] == 2:
                    kfs2_by_the_way.append([step.x+self.DIRCTIONS[step.yaw][0], step.y+self.DIRCTIONS[step.yaw][1]])
                    grid_cnt[step.x+self.DIRCTIONS[step.yaw][0]][step.y+self.DIRCTIONS[step.yaw][1]] = 0
        
        kfs2_can_get = []
        for step in self.eval_path:
            for direction in self.DIRCTIONS:
                if 0<= step.x+direction[0] <= 5 and 0<= step.y+direction[1] <= 2:
                    if grid_cnt[step.x+direction[0]][step.y+direction[1]] == 2:
                        kfs2_can_get.append([step.x+direction[0], step.y+direction[1]])
                        grid_cnt[step.x+direction[0]][step.y+direction[1]] = 0
        
        if len(kfs2_on_the_way) + len(kfs2_by_the_way) + len(kfs2_can_get) < 2: return
        
        kfs1_on_the_way = []
        for step in self.eval_path:
            if grid_cnt[step.x][step.y] == 1:
                kfs1_on_the_way.append([step.x-1, step.y])
                grid_cnt[step.x][step.y] = 0
        
        cost=0
        kfs2_needed=0
        kfs2_by_the_way_needed = 2-len(kfs2_on_the_way)
        if kfs2_by_the_way_needed >len(kfs2_by_the_way):
            kfs2_by_the_way_needed = len(kfs2_by_the_way)
        if kfs2_by_the_way_needed <0:
            kfs2_by_the_way_needed = 0
        kfs2_can_get_needed = 2-len(kfs2_on_the_way)-kfs2_by_the_way_needed
        if kfs2_can_get_needed < 0:
            kfs2_can_get_needed = 0
        while len(kfs2_by_the_way) > kfs2_by_the_way_needed:
            kfs2_by_the_way.pop()
        while len(kfs2_can_get) > kfs2_can_get_needed:
            kfs2_can_get.pop()
        kfs2_needed = len(kfs2_on_the_way) + kfs2_by_the_way_needed + kfs2_can_get_needed
        
        pub_path = []
        for i in range(len(self.eval_path)):
            step = self.eval_path[i]
            if i==0: pub_path.append(Step(step.x-1, step.y, step.yaw, True, step.send_can_do))
            else: pub_path.append(Step(step.x-1, step.y, step.yaw, False, step.send_can_do))

            exStep=[]
            flag1=False
            flag2=False
            next_step = self.eval_path[i+1] if i+1<len(self.eval_path) else None
            if len(kfs2_on_the_way)>0 and next_step is not None and kfs2_on_the_way[0] == [next_step.x, next_step.y]:
                exStep.append(Step(step.x-1, step.y, next_step.yaw, False, True))
                kfs2_on_the_way.pop(0)
                flag1=True
            if len(kfs2_by_the_way)>0 and 0<= step.x+self.DIRCTIONS[step.yaw][0] <= 5 and 0<= step.y+self.DIRCTIONS[step.yaw][1] <= 2:
                if kfs2_by_the_way[0] == [step.x+self.DIRCTIONS[step.yaw][0], step.y+self.DIRCTIONS[step.yaw][1]]:
                    exStep.insert(0, Step(step.x-1, step.y, step.yaw, False, True))
                    kfs2_by_the_way.pop(0)
                    flag2=True
            for direction in self.DIRCTIONS:
                if len(kfs2_can_get)>0 and 0<= step.x+direction[0] <= 5 and 0<= step.y+direction[1] <= 2:
                    if kfs2_can_get[0] == [step.x+direction[0], step.y+direction[1]]:
                        if(flag1 and (not flag2)):exStep.insert(0, Step(step.x-1, step.y, direction[2], False, True))
                        else: exStep.append(Step(step.x-1, step.y, direction[2], False, True))
                        kfs2_can_get.pop(0)
                        break
            pub_path.extend(exStep)
        
        for i in range(len(pub_path)):
            step = pub_path[i]
            pre_step = pub_path[i-1] if i-1>=0 else None
            if pre_step is not None and step.yaw != pre_step.yaw:
                cost += self.TURN_COST
            if pre_step is not None and pre_step.send_can_do:
                pub_path[i] = Step(step.x, step.y, step.yaw, True, step.send_can_do)
            if pre_step is not None and (pre_step.x != step.x or pre_step.y != step.y):
                cost += self.MOVE_COST
            if step.send_can_do:
                cost += self.ADJACENT_COST+self.FETCH_KFS2_COST
        self._add_path(pub_path, cost, kfs2_needed, kfs1_on_the_way)
    
    def _add_path(self, path, cost, kfs2_needed, kfs1_affected):
        self.path[kfs2_needed].append((path, cost, kfs2_needed, kfs1_affected))
        pos = len(self.path[kfs2_needed])-1
        i=len(self.path[kfs2_needed])-1
        while i>=0 and self.path[kfs2_needed][i][1] > self.path[kfs2_needed][pos][1]:i-=1
        while i>=0 and self.path[kfs2_needed][i][1] == self.path[kfs2_needed][pos][1] and len(self.path[kfs2_needed][i][3]) > len(self.path[kfs2_needed][pos][3]):i-=1
        self.path[kfs2_needed].insert(i+1, self.path[kfs2_needed].pop(pos))
        
            
        