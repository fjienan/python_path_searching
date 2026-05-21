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
from core.grid_utils import GridConverter

class DFSPlanner:
    def __init__(self, grid_cols, grid_rows, start_x, start_y, reserved_length, grid_converter, kfs_grid_height, logger=None):
        self.logger = logger
        self.GRID_ROWS = grid_rows
        self.GRID_COLS = grid_cols
        self.KFS2_PREFFERRED = 1
        self.initial_pos_x = start_x
        self.initial_pos_y = start_y
        self.logger.info(f'start position ({self.initial_pos_x}, {self.initial_pos_y})')
        self.DIRCTIONS = [(1, 0, 0), (0, 1, math.pi/2), (0, -1, -math.pi/2)] 
        self.GRID = np.zeros((self.GRID_ROWS+2, self.GRID_COLS), dtype=int)
        self.RESERVERED_LENGTH = reserved_length
        self.visited = np.zeros((self.GRID_ROWS+2, self.GRID_COLS), dtype=bool)
        self.path = [[],[],[],[]]
        self.cnt_path = []
        self.eval_path=[]
        
        self.MOVE_COST = 6
        self.TURN_COST = 2
        self.ADJACENT_COST = 1
        self.FETCH_KFS2_COST = 6
        
        self.grid_converter = grid_converter
        self.kfs_grid_height = kfs_grid_height
    
    def plan_path(self, grid, stx, sty):
        """DFS路径规划算法实现"""
        self.GRID = np.vstack(([0, 0, 0], grid, [0, 0, 0]))
        # self.logger.info(f"GRID:\n{self.GRID}")
        self._dfs(stx+1, sty, 0)
        self.logger.info(f'Found {len(self.path[0])} paths with 0 kfs2 needed, {len(self.path[1])} paths with 1 kfs2 needed, {len(self.path[2])} paths with 2 kfs2 needed, {len(self.path[3])} paths with 3 kfs2 needed')
        if len(self.path[1]) ==0:
            return None
        return self.path[1][0]
        """ if len(self.path[2])==0 and len(self.path[3])==0:
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
                elif len(self.path[2][0][3]) < len(self.path[3][0][3]):
                    return self.path[3][0]
                else:
                    return self.path[2][0] """

    def _dfs(self, posX, posY, yaw):
        
        # self.logger.info(f'Visiting ({posX}, {posY}), yaw={yaw}')
        if posX==5:
            self.cnt_path.append(Step(posX, posY, yaw))
            self._copy_path(self.cnt_path, self.eval_path)
            self._evaluate_path()
            self.cnt_path.pop()
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
            if posX == 0 and newX == 1 and newY != 1:
                continue
            # self.logger.info(f'At ({posX}, {posY}), trying to move to ({newX}, {newY}) with direction {direction[2]}')
           
            self._dfs(newX, newY, direction[2])
        self.visited[posX][posY] = False
        self.cnt_path.pop()
    
    def _copy_path(self, src, dst):
        dst.clear()
        for step in src:
            dst.append(Step(step.x, step.y, step.yaw, step.require_can_go, step.send_can_do))
    
    def _get_dir_idx(self, yaw):
        if abs(yaw-0)<1e-6: return 0
        elif abs(yaw-math.pi/2)<1e-6: return 1
        elif abs(yaw+math.pi/2)<1e-6: return 2
        return 0

    def _evaluate_path(self):
        first_row_needed = -1
        kfs2_needed=0
        if 2 in self.GRID[1]:
            if self.GRID[1][1] == 2: first_row_needed = 1
            elif self.GRID[1][0] == 2:
                first_row_needed = 0
                kfs2_needed=1
            elif self.GRID[1][2] == 2:
                first_row_needed = 2
                kfs2_needed=1
        grid_cnt = self.GRID.copy()
        #self.logger.info(f'{grid_cnt}')
        kfs2_on_the_way = []
        #for step in self.eval_path:
            #self.logger.info(f'step: x={step.x}, y={step.y}, yaw={step.yaw}, require_can_go={step.require_can_go}, send_can_do={step.send_can_do}')
        for step in self.eval_path:
            nowX, nowY = round(step.x), round(step.y)
            if grid_cnt[nowX][nowY] == 2:
                kfs2_on_the_way.append([nowX, nowY])
                grid_cnt[nowX][nowY] = 0
                
        # if len(kfs2_on_the_way) == 4: return
        # modify for the claw error
        if len(kfs2_on_the_way) > 1: return
        if len(kfs2_on_the_way) == 1 and first_row_needed in [0,2]: return
        
        kfs2_by_the_way = []
        for step in self.eval_path:
            now_dir = self._get_dir_idx(step.yaw)
            nowX, nowY = round(step.x), round(step.y)
            if 0<= nowX+self.DIRCTIONS[now_dir][0] <= 5 and 0<= nowY+self.DIRCTIONS[now_dir][1] <= 2:
                if grid_cnt[nowX+self.DIRCTIONS[now_dir][0]][nowY+self.DIRCTIONS[now_dir][1]] == 2:
                    kfs2_by_the_way.append([nowX+self.DIRCTIONS[now_dir][0], nowY+self.DIRCTIONS[now_dir][1]])
                    grid_cnt[nowX+self.DIRCTIONS[now_dir][0]][nowY+self.DIRCTIONS[now_dir][1]] = 0
        
        kfs2_can_get = []
        for step in self.eval_path:
            nowX, nowY = round(step.x), round(step.y)
            for direction in self.DIRCTIONS:
                if 0<= nowX+direction[0] <= 5 and 0<= nowY+direction[1] <= 2:
                    if grid_cnt[nowX+direction[0]][nowY+direction[1]] == 2:
                        kfs2_can_get.append([nowX+direction[0], nowY+direction[1]])
                        grid_cnt[nowX+direction[0]][nowY+direction[1]] = 0
        
        # if len(kfs2_on_the_way) + len(kfs2_by_the_way) + len(kfs2_can_get) < 2: return
        # modify for the claw error
        if first_row_needed == -1 and len(kfs2_on_the_way) + len(kfs2_by_the_way) + len(kfs2_can_get) <1: return
        # if first_row_needed in [0,2] and len(kfs2_on_the_way) + len(kfs2_by_the_way) + len(kfs2_can_get) >0: return
        
        kfs1_on_the_way = []
        for step in self.eval_path:
            nowX, nowY = round(step.x), round(step.y)
            if grid_cnt[nowX][nowY] == 1:
                kfs1_on_the_way.append([nowX-1, nowY])
                grid_cnt[nowX][nowY] = 0
        
        cost=0;kfs2_by_the_way_needed=0;kfs2_can_get_needed=0
        if kfs2_needed == 0:
            kfs2_by_the_way_needed = self.KFS2_PREFFERRED-len(kfs2_on_the_way)
            if kfs2_by_the_way_needed >len(kfs2_by_the_way):
                kfs2_by_the_way_needed = len(kfs2_by_the_way)
            if kfs2_by_the_way_needed <0:
                kfs2_by_the_way_needed = 0
            kfs2_can_get_needed = self.KFS2_PREFFERRED-len(kfs2_on_the_way)-kfs2_by_the_way_needed
            if kfs2_can_get_needed < 0:
                kfs2_can_get_needed = 0
            while len(kfs2_by_the_way) > kfs2_by_the_way_needed:
                kfs2_by_the_way.pop()
            while len(kfs2_can_get) > kfs2_can_get_needed:
                kfs2_can_get.pop()
            kfs2_needed += len(kfs2_on_the_way) + kfs2_by_the_way_needed + kfs2_can_get_needed
        else:
            while len(kfs2_by_the_way) > kfs2_by_the_way_needed:
                kfs2_by_the_way.pop()
            while len(kfs2_can_get) > kfs2_can_get_needed:
                kfs2_can_get.pop()
        


        pub_path = []
        start_x_to_origin, start_y_to_origin = self.grid_converter.grid_to_map(round(self.eval_path[0].x-1), round(self.eval_path[0].y))
        self.logger.info(f'start_x_to_origin={start_x_to_origin}, start_y_to_origin={start_y_to_origin}')
        self.logger.info(f'initial_pos_x={self.initial_pos_x}, initial_pos_y={self.initial_pos_y}')
        # cope with first line kfs2
        if first_row_needed == 0:
            nowX, nowY = self.grid_converter.grid_to_map(-1, 0)
            nowX_to_r2 = nowX - start_x_to_origin + self.initial_pos_x
            nowY_to_r2 = nowY - start_y_to_origin + self.initial_pos_y
            nowX_to_r2_consider_yaw = nowX_to_r2-math.cos(-math.pi/2)*self.RESERVERED_LENGTH
            nowY_to_r2_consider_yaw = nowY_to_r2-math.sin(-math.pi/2)*self.RESERVERED_LENGTH
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, -math.pi/2, True, 0))
            nowX_to_r2_consider_yaw -= self.RESERVERED_LENGTH
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, -math.pi/2, False, 0))
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, 0, False, 0))
            nowY_to_r2_consider_yaw -= self.RESERVERED_LENGTH
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, 0, False, 400))
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, 0, False, 0))
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, math.pi/2, False, 0))
            nowX, nowY = self.grid_converter.grid_to_map(-1, 1)
            nowX_to_r2 = nowX - start_x_to_origin + self.initial_pos_x
            nowY_to_r2 = nowY - start_y_to_origin + self.initial_pos_y
            nowX_to_r2_consider_yaw = nowX_to_r2-self.RESERVERED_LENGTH
            nowY_to_r2_consider_yaw = nowY_to_r2
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, math.pi/2, False, 0))
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, 0, False, 0))
        if first_row_needed == 2:
            nowX, nowY = self.grid_converter.grid_to_map(-1, 2)
            nowX_to_r2 = nowX - start_x_to_origin + self.initial_pos_x
            nowY_to_r2 = nowY - start_y_to_origin + self.initial_pos_y
            nowX_to_r2_consider_yaw = nowX_to_r2-math.cos(math.pi/2)*self.RESERVERED_LENGTH
            nowY_to_r2_consider_yaw = nowY_to_r2-math.sin(math.pi/2)*self.RESERVERED_LENGTH
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, math.pi/2, True, 0))
            nowX_to_r2_consider_yaw -= self.RESERVERED_LENGTH
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, math.pi/2, False, 0))
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, 0, False, 0))
            nowY_to_r2_consider_yaw += self.RESERVERED_LENGTH
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, 0, False, 400))
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, 0, False, 0))
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, -math.pi/2, False, 0))
            nowX, nowY = self.grid_converter.grid_to_map(-1, 1)
            nowX_to_r2 = nowX - start_x_to_origin + self.initial_pos_x
            nowY_to_r2 = nowY - start_y_to_origin + self.initial_pos_y
            nowX_to_r2_consider_yaw = nowX_to_r2-self.RESERVERED_LENGTH
            nowY_to_r2_consider_yaw = nowY_to_r2
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, -math.pi/2, False, 0))
            pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, 0, False, 0))

        for i in range(len(self.eval_path)):
            step = self.eval_path[i]
            nowX, nowY = self.grid_converter.grid_to_map(round(step.x-1), round(step.y))
            nowX_to_r2 = nowX - start_x_to_origin + self.initial_pos_x
            nowY_to_r2 = nowY - start_y_to_origin + self.initial_pos_y
            # self.logger.info(f'nowX_to_r2={nowX_to_r2}, nowY_to_r2={nowY_to_r2}')
            nowX_to_r2_consider_yaw = nowX_to_r2-math.cos(step.yaw)*self.RESERVERED_LENGTH
            nowY_to_r2_consider_yaw = nowY_to_r2-math.sin(step.yaw)*self.RESERVERED_LENGTH
            if i==0: pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, step.yaw, True, step.send_can_do))
            else: pub_path.append(Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, step.yaw, False, step.send_can_do))

            exStep=[]
            flag1=False
            flag2=False
            next_step = self.eval_path[i+1] if i+1<len(self.eval_path) else None
            if len(kfs2_on_the_way)>0 and next_step is not None and kfs2_on_the_way[0] == [round(next_step.x), round(next_step.y)]:
                nexX_to_r2_consider_yaw = nowX_to_r2-math.cos(next_step.yaw)*self.RESERVERED_LENGTH
                nexY_to_r2_consider_yaw = nowY_to_r2-math.sin(next_step.yaw)*self.RESERVERED_LENGTH
                height_diff = self.kfs_grid_height[round(next_step.x-1)][round(next_step.y)] - self.kfs_grid_height[round(step.x-1)][round(step.y)]
                exStep.append(Step(nexX_to_r2_consider_yaw, nexY_to_r2_consider_yaw, next_step.yaw, False, height_diff))
                kfs2_on_the_way.pop(0)
                flag1=True
            now_dir = self._get_dir_idx(step.yaw)
            if len(kfs2_by_the_way)>0 and 0 <= round(step.x)+self.DIRCTIONS[now_dir][0] <= 5 and 0<= round(step.y)+self.DIRCTIONS[now_dir][1] <= 2:
                if kfs2_by_the_way[0] == [round(step.x)+self.DIRCTIONS[now_dir][0], round(step.y)+self.DIRCTIONS[now_dir][1]]:
                    height_diff = self.kfs_grid_height[round(step.x-1)+self.DIRCTIONS[now_dir][0]][round(step.y)+self.DIRCTIONS[now_dir][1]] - self.kfs_grid_height[round(step.x-1)][round(step.y)]
                    exStep.insert(0, Step(nowX_to_r2_consider_yaw, nowY_to_r2_consider_yaw, step.yaw, False, height_diff))
                    kfs2_by_the_way.pop(0)
                    flag2=True
            for direction in self.DIRCTIONS:
                if len(kfs2_can_get)>0 and 0<= round(step.x)+direction[0] <= 5 and 0<= round(step.y)+direction[1] <= 2:
                    if kfs2_can_get[0] == [round(step.x)+direction[0], round(step.y)+direction[1]]:
                        nexX_to_r2_consider_yaw = nowX_to_r2-math.cos(direction[2])*self.RESERVERED_LENGTH
                        nexY_to_r2_consider_yaw = nowY_to_r2-math.sin(direction[2])*self.RESERVERED_LENGTH
                        height_diff = self.kfs_grid_height[round(step.x-1)+direction[0]][round(step.y)+direction[1]] - self.kfs_grid_height[round(step.x-1)][round(step.y)]
                        if(flag1 and (not flag2)):exStep.insert(0, Step(nexX_to_r2_consider_yaw, nexY_to_r2_consider_yaw, direction[2], False, height_diff))
                        else: exStep.append(Step(nexX_to_r2_consider_yaw, nexY_to_r2_consider_yaw, direction[2], False, height_diff))
                        kfs2_can_get.pop(0)
                        break
            pub_path.extend(exStep)
        
        # self.logger.info("Here----------------------------")
        idx=1
        if first_row_needed in [0,2]: idx+=8
        while idx<len(pub_path):
            if pub_path[idx-1].yaw != pub_path[idx].yaw:
                if abs(pub_path[idx-1].x+self.RESERVERED_LENGTH*math.cos(pub_path[idx-1].yaw) - (pub_path[idx].x+self.RESERVERED_LENGTH*math.cos(pub_path[idx].yaw))) > 1e-6 or abs(pub_path[idx-1].y+self.RESERVERED_LENGTH*math.sin(pub_path[idx-1].yaw) - (pub_path[idx].y+self.RESERVERED_LENGTH*math.sin(pub_path[idx].yaw))) > 1e-6:
                    pub_path.insert(idx, Step(pub_path[idx-1].x+self.RESERVERED_LENGTH*math.cos(pub_path[idx-1].yaw)-math.cos(pub_path[idx].yaw)*self.RESERVERED_LENGTH,
                                              pub_path[idx-1].y+self.RESERVERED_LENGTH*math.sin(pub_path[idx-1].yaw)-math.sin(pub_path[idx].yaw)*self.RESERVERED_LENGTH,
                                              pub_path[idx].yaw, False, 0))
                    idx-=1
                elif abs(pub_path[idx-1].yaw)<1e-6:
                    if abs(pub_path[idx].yaw-math.pi/2)<1e-6:
                        pub_path.insert(idx, Step(pub_path[idx-1].x, pub_path[idx-1].y-self.RESERVERED_LENGTH, pub_path[idx-1].yaw, False, 0))
                    else:
                        pub_path.insert(idx, Step(pub_path[idx-1].x, pub_path[idx-1].y+self.RESERVERED_LENGTH, pub_path[idx-1].yaw, False, 0))
                    idx+=1
                elif abs(abs(pub_path[idx].yaw-pub_path[idx-1].yaw)-math.pi)<1e-6:
                    pub_path.insert(idx, Step(pub_path[idx-1].x-self.RESERVERED_LENGTH, (pub_path[idx-1].y+pub_path[idx].y)*0.5, 0, False, 0))
                else:
                    if abs(pub_path[idx-1].yaw-math.pi/2)<1e-6:
                        pub_path.insert(idx, Step(pub_path[idx-1].x-self.RESERVERED_LENGTH, pub_path[idx-1].y, pub_path[idx-1].yaw, False, 0))
                    else:
                        pub_path.insert(idx, Step(pub_path[idx-1].x-self.RESERVERED_LENGTH, pub_path[idx-1].y, pub_path[idx-1].yaw, False, 0))
                    
                    idx+=1
            idx+=1
        
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
        # self.logger.info(f'Evaluated path with cost {cost}, kfs2_needed={kfs2_needed}, kfs1_affected={kfs1_affected}')
        self.path[kfs2_needed].append((path, cost, kfs2_needed, kfs1_affected))
        pos = len(self.path[kfs2_needed])-1
        i=len(self.path[kfs2_needed])-1
        while i>=0 and self.path[kfs2_needed][i][1] > self.path[kfs2_needed][pos][1]:i-=1
        while i>=0 and self.path[kfs2_needed][i][1] == self.path[kfs2_needed][pos][1] and len(self.path[kfs2_needed][i][3]) > len(self.path[kfs2_needed][pos][3]):i-=1
        self.path[kfs2_needed].insert(i+1, self.path[kfs2_needed].pop(pos))
        if i == 0:
            self.logger.info("Found best path so far! Details:")
            self.logger.info("--- cnt_path ---")
            for idx, s in enumerate(self.cnt_path):
                self.logger.info(f"  Step {idx}: x={s.x}, y={s.y}, yaw={s.yaw:.2f}, req_go={s.require_can_go}, send_do={s.send_can_do}")
            
            self.logger.info("--- eval_path ---")
            for idx, s in enumerate(self.eval_path):
                self.logger.info(f"  Step {idx}: x={s.x}, y={s.y}, yaw={s.yaw:.2f}, req_go={s.require_can_go}, send_do={s.send_can_do}")
            
            self.logger.info("--- pub_path ---")
            for idx, s in enumerate(path):
                self.logger.info(f"  Step {idx}: x={s.x:.2f}, y={s.y:.2f}, yaw={s.yaw:.2f}, req_go={s.require_can_go}, send_do={s.send_can_do}")
            
        
            
        