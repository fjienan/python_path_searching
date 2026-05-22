#!/usr/bin/env python3
"""
轨迹跟踪节点

状态机流程：
  单向/全向模式：HOLD → MOVE → ADJ(if precision needed) → HOLD(if can_go required) → MOVE...
           - HOLD: 停在关键点，等 can_go=True 才继续
           - MOVE: 角度大于阈值时先修正，直线前进到目标点，实时校正偏离直线误差、角度误差（阈值较大）
           - ADJ: 如需精细化对齐才进入（阈值较小），如果在某点不需要抓取kfs操作则不会进入 ADJ，以加快运行速度。

  全向模式获得的路径无需旋转对齐，直接控制 x/y 轴速度即可；单向模式需要先旋转对齐，再控制前进速度，侧向误差较大时也会进行侧向修正。

"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32, Float32, Float32MultiArray, String
import json
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import math
import numpy as np

from core.pid_controller import PIDController
from robot_pose.transformer import PoseTransformerQuat
from core.transform_utils import yaw_from_quaternion

from rclpy.action import ActionClient

from bt_state_interfaces.action import SwitchNavigationState


class TrackerNode(Node):

    ADJ, MOVE, HOLD, FETCH, RET = 0, 1, 2, 3, 4

    def __init__(self):
        super().__init__('tracker')
        
        # --- 地图参数 ---
        self.declare_parameter('map_origin',        [3.2, 1.2, 0.0])

        # --- 运动模式 ---
        self.declare_parameter('motion_type',        'unidirectional')
        self.declare_parameter('control_frequency',  50.0)
        self.declare_parameter('arrive_threshold',    0.12)
        self.declare_parameter('angle_threshold',    0.08)
        self.declare_parameter('arrive_precise_threshold',    0.01)
        self.declare_parameter('angle_precise_threshold',    0.01)
        self.declare_parameter('reserved_length', 0.15)

        # --- Topics ---
        self.declare_parameter('planning_path',       '/planning/path')
        self.declare_parameter('initial_pose',       '/initial_pose')
        self.declare_parameter('odom_world',         '/odom_world')
        self.declare_parameter('can_go',            '/can_go')
        self.declare_parameter('cmd_vel',            '/cmd_vel')
        self.declare_parameter('tracker_direction',  '/direction')
        self.declare_parameter('target_yaw_deg',      '/target_yaw_deg')
        self.declare_parameter('suspension_state', '/current_state')
        self.declare_parameter('suspension_state_pub', '/current_state')
        self.declare_parameter('suspension_control', '/t0x0102_action')
        self.declare_parameter('fetch_done', '/t0x0103_')
        self.declare_parameter('can_do', '/can_do')

        # --- PID ---
        self.declare_parameter('kp_angle', 4.0); self.declare_parameter('ki_angle', 0.0); self.declare_parameter('kd_angle', 0.2)
        self.declare_parameter('kp_dist',  2.5); self.declare_parameter('ki_dist',  0.0); self.declare_parameter('kd_dist',  0.1)
        self.declare_parameter('kp_adj_angle', 3.0); self.declare_parameter('ki_adj_angle', 0.0); self.declare_parameter('kd_adj_angle', 0.1)
        self.declare_parameter('kp_adj_dist',  2.0); self.declare_parameter('ki_adj_dist',  0.0); self.declare_parameter('kd_adj_dist',  0.1)
        self.declare_parameter('max_vel', 0.5)
        self.declare_parameter('max_angular', 2.0)
        self.declare_parameter('max_adj_angular', 1.0)
        self.declare_parameter('max_adj_vel', 0.5)
        


        self.map_origin          = tuple(self._p('map_origin'))
        self.motion_type         = self._p('motion_type')
        self.control_frequency   = self._p('control_frequency')
        self.arrive_threshold    = self._p('arrive_threshold')
        self.angle_threshold     = self._p('angle_threshold')
        self.arrive_precise_threshold = self._p('arrive_precise_threshold')
        self.angle_precise_threshold = self._p('angle_precise_threshold')
        self.reserved_length     = self._p('reserved_length')

        self.get_logger().info(f'__init__: Tracker Node started, motion_type={self.motion_type}')

        self.current_path          = None
        self.current_target_index  = 0
        self.current_pos           = np.array([0.0, 0.0, 0.0])
        self.current_yaw           = 0.0
        self.target_yaw            = 0.0
        self.have_path             = False
        self.have_odom             = False
        self.can_go                = False
        self.angle_ready           = False
        self.start_pos_x = 0
        self.start_pos_y = 0
        # 直线闭环状态变量
        self._move_start_pos = np.array([0.0, 0.0])

        self.state = self.HOLD
        self.state_switched = False
        
        self.suspension_state = 0
        self.navigation_state_client = ActionClient(
            self,
            SwitchNavigationState,
            '/switch_navigation_state',
        )

        self._init_pids()
        self._init_subscribers()
        self._init_publishers()

        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, self._control_callback
        )

    def _p(self, name, default=None):
        val = self.get_parameter(name).value
        return val if val is not None else default

    def _init_pids(self):
        self.pid_angle = PIDController(self._p('kp_angle'), self._p('ki_angle'), self._p('kd_angle'), self._p('max_angular'))
        self.pid_dist  = PIDController(self._p('kp_dist'),  self._p('ki_dist'),  self._p('kd_dist'),  self._p('max_vel'))
        self.pid_side  = PIDController(self._p('kp_dist'),  self._p('ki_dist'),  self._p('kd_dist'),  self._p('max_vel'))
        self.pid_adj_angle = PIDController(self._p('kp_adj_angle'), self._p('ki_adj_angle'), self._p('kd_adj_angle'), self._p('max_adj_angular'))
        self.pid_adj_dist  = PIDController(self._p('kp_adj_dist'),  self._p('ki_adj_dist'),  self._p('kd_adj_dist'),  self._p('max_adj_vel'))
        self.pid_adj_side  = PIDController(self._p('kp_adj_dist'),  self._p('ki_adj_dist'),  self._p('kd_adj_dist'),  self._p('max_adj_vel'))
        
    def _init_subscribers(self):
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_sub   = self.create_subscription(String,    self._p('planning_path', '/planning/path'), self._path_callback,  qos)
        self.odom_sub   = self.create_subscription(PoseStamped, self._p('odom_world',   '/odom_world'),   self._odom_callback,   10)
        self.can_go_sub = self.create_subscription(Bool,     self._p('can_go',       '/can_go'),       self._can_go_callback, 10)
        self.fetch_down_sub = self.create_subscription(Float32MultiArray, self._p('fetch_done', '/t0x0103_'), self._fetch_done_callback, 10)
        self.suspension_state_sub = self.create_subscription(Int32, self._p('suspension_state', '/current_state'), self._suspension_state_callback, 10)

    def _init_publishers(self):
        self.cmd_pub = self.create_publisher(Float32MultiArray, self._p('cmd_vel', '/t0x0101'), 10)
        self.dir_pub = self.create_publisher(Int32,    self._p('tracker_direction',  '/direction'),         10)
        self.yaw_pub = self.create_publisher(Float32,  self._p('target_yaw_deg',    '/target_yaw_deg'),    10)
        self.initPos_pub = self.create_publisher(Float32MultiArray, self._p('initial_pose', '/initial_pose'), 10)
        self.suspension_state_pub = self.create_publisher(Int32, self._p('suspension_state_pub', '/target_state'), 10)
        self.suspension_control_pub = self.create_publisher(Float32MultiArray, self._p('suspension_control', '/t0x0102_action'), 10)
        self.can_do_pub = self.create_publisher(Float32MultiArray, self._p('can_do', '/can_do'), 10)

    def _fetch_done_callback(self, msg):
        if abs(msg.data[0]+1)<1e-6:
            self.get_logger().info(f'_fetch_done_callback: fetch done signal received, can_do=1')
            self.can_go = True
    
    def _suspension_state_callback(self, msg):
        self.suspension_state = msg.data

    def _path_callback(self, msg):
        try:
            path_data = json.loads(msg.data)
            points = path_data.get("points", [])
        except Exception as e:
            self.get_logger().error(f'_path_callback: Failed to parse path JSON: {e}')
            return

        if len(points) == 0:
            self.get_logger().warn('_path_callback: Received empty path')
            return

        new_target_idx = 1
        if new_target_idx >= len(points):
            self.get_logger().warn('_path_callback: Path too short after skipping first point')
            return

        if self.current_path is not None and self.have_path:
            cur_target = self._get_current_target()
            if cur_target is not None:
                new_target = np.array([
                    points[new_target_idx]["x"] + self.start_pos_x,
                    points[new_target_idx]["y"] + self.start_pos_y
                ])
                if np.allclose(cur_target, new_target, atol=0.05):
                    self.current_path = points
                    return

        self.current_path = points
        self.get_logger().info(f'_path_callback: Received path with {len(points)} points')
        for i, p in enumerate(self.current_path):
            ox = p["x"]
            oy = p["y"]
            ori = self._normalize_angle(p["yaw"])
            self.get_logger().info(f'_path_callback: Path point {i}: x={ox:.3f}, y={oy:.3f}, yaw={math.degrees(ori)} deg')

        self.current_target_index   = new_target_idx
        self.have_path              = True
        self.state                  = self.HOLD
        self.angle_ready             = False
        self._reset_pids()
        self._read_target_yaw()
        self._publish_direction()
        self.yaw_pub.publish(Float32(data=math.degrees(self.target_yaw)))
        self.get_logger().info(f'_path_callback: new_idx={self.current_target_index} '
            f'total={len(self.current_path)}, entering HOLD')
        
    def _odom_callback(self, msg):
        self.current_pos[0] = msg.pose.position.x
        self.current_pos[1] = msg.pose.position.y
        self.current_pos[2] = msg.pose.position.z
        self.quaterion = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        self.current_yaw = self._normalize_angle(yaw_from_quaternion(self.quaterion))
        if not self.have_odom:
            self.start_pos_x = self.current_pos[0]
            self.start_pos_y = self.current_pos[1]
            self.have_odom = True
            self.initPos_pub.publish(Float32MultiArray(data=[self.start_pos_x, self.start_pos_y]))
        #self.get_logger().info(f"pos=({self.current_pos[0]},{self.current_pos[1]}, {math.degrees(self.current_yaw)})")
    def _can_go_callback(self, msg):
        self.can_go = msg.data

    def _read_target_yaw(self):
        if self.current_path is None or self.current_target_index >= len(self.current_path): return
        self.target_yaw = self._normalize_angle(self.current_path[self.current_target_index]["yaw"])

    def _reset_pids(self):
        self.pid_angle.reset(); self.pid_dist.reset(); self.pid_side.reset()

    def _publish_direction(self):
        if self._p("motion_type")=="unidirectional":
             self.dir_pub.publish(Int32(data=0))
             self.get_logger().info(f'_publish_direction: motion_type=unidirectional, force direction=0')
             return
         
        direction = 0
        tar_yaw_deg  = math.degrees(self.target_yaw)
        if tar_yaw_deg > 2.0: direction = -1
        elif tar_yaw_deg < -2.0: direction = 1
        else: direction = 0
        self.dir_pub.publish(Int32(data=direction))
        self.get_logger().info(
            f'_publish_direction: motion_type={self.motion_type}, target_yaw={tar_yaw_deg:.1f} deg, '
            f'direction={direction}'
        )

    def _normalize_angle(self, angle):
        while angle > math.pi:  angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def _get_current_target(self):
        if self.current_path is None or self.current_target_index >= len(self.current_path):
            return None
        p = self.current_path[self.current_target_index]
        #return np.array([p["x"]+self.start_pos_x, p["y"]+self.start_pos_y, p["require_can_go"], p["send_can_do"]])
        return np.array([p["x"], p["y"], p["require_can_go"], p["send_can_do"]])

    def _move_to_next_target(self):
        self.current_target_index += 1
        self.can_go = False
        self.angle_ready = False
        target = self._get_current_target()
        self.get_logger().info(
            f'_move_to_next_target: new_idx={self.current_target_index} '
            f'total={len(self.current_path)}, angle_ready={self.angle_ready}, entering HOLD'
        )
        if self.current_target_index >= len(self.current_path):
            self.get_logger().info('_move_to_next_target: All targets reached, stopping')
            return False
        self._read_target_yaw()
        self._reset_pids()
        self.state=self.MOVE
        self._publish_direction()
        self.yaw_pub.publish(Float32(data=math.degrees(self.target_yaw)))
        return True

    def _control_callback(self):
        if not self.have_path or not self.have_odom:
            self._stop()
            return

        target = self._get_current_target()
        if target is None:
            self._stop()
            self.have_path = False
            return

        if self.motion_type == 'omnidirectional': self._control_omni(target)
        else: self._control_uni(target)

    # =======================================================================
    #  单向模式
    # =======================================================================
    def _control_uni(self, target):
        
        dt = 1.0 / self.control_frequency

        if self.state == self.HOLD:
            
            self._stop()
            
            if not self.can_go:
                return 
            
            self._reset_pids()
            
            self.state = self.MOVE
            self.get_logger().info(f'HOLD: can_go=True,angle_ready = {self.angle_ready}, entering MOVE')
            
            # self.get_logger().info(
            #     f'MOVE: angle_ready={self.angle_ready}, pos={self.current_pos[0]:.2f},{self.current_pos[1]:.2f},{math.degrees(self.current_yaw):.1f} deg '
            #     f'target={target[0]:.2f},{target[1]:.2f},{math.degrees(self.target_yaw):.1f} deg '
            # )

        elif self.state == self.MOVE:
            
            # 读取当前参数
            self._read_target_yaw()
            dx   = target[0] - self.current_pos[0]
            dy   = target[1] - self.current_pos[1]
            angle_error = self._normalize_angle(self.target_yaw - self.current_yaw)
            


            # 终止条件检测
            arrived = abs(dx) < self.arrive_threshold and \
                      abs(dy) < self.arrive_threshold and \
                      abs(angle_error) < self.angle_threshold
            if arrived:
                self.get_logger().info(
                    f'MOVE: roughly arrived! idx={self.current_target_index}/'
                )
                self.state = self.ADJ
                self.get_logger().info(f'{len(self.current_path)}, entering ADJ to precisely align.')
                return       
            
            # 未到终点，进行修正，将位置修正与角度修正分开处理
            vel_forward,vel_side,vel_angle = 0,0,0
            if abs(angle_error) >= self.angle_threshold and self.suspension_state in [-1,0,14,21]:
                vel_angle = self.pid_angle.compute(angle_error, dt)
                if vel_angle > 0: vel_angle = max(0.15, min(self._p('max_angular'), vel_angle))
                elif vel_angle < 0: vel_angle = min(-0.15, max(-self._p('max_angular'), vel_angle))
            else: self.angle_ready = True

            # 设置速度
            # - 计算距离
            if(abs(math.degrees(self.target_yaw))<1e-6): dis_forward,dis_side = dx, dy
            elif(abs(math.degrees(self.target_yaw)-90)<1e-6): dis_forward,dis_side = dy, -dx
            elif(abs(math.degrees(self.target_yaw)+90)<1e-6): dis_forward,dis_side = -dy, dx
            else: dis_forward,dis_side = -dx, -dy

            # - 轴向
            vel_forward = self.pid_dist.compute(dis_forward, dt)
            if abs(dis_forward) >= self.arrive_threshold:
                if vel_forward > 0: vel_forward = max(0.15, min(self._p('max_vel'), vel_forward))
                elif vel_forward < 0: vel_forward = min(-0.15, max(-self._p('max_vel'), vel_forward))
            else: vel_forward = 0.0
            
            # - 侧向
            vel_side = self.pid_side.compute(dis_side, dt)
            if abs(dis_side) >= self.arrive_threshold:
                if vel_side > 0: vel_side = max(0.15, min(self._p('max_vel'), vel_side))
                elif vel_side < 0: vel_side = min(-0.15, max(-self._p('max_vel'), vel_side))
            else: vel_side = 0.0
            
            if not self.angle_ready:
                self._publish_cmd(0, 0, vel_angle)
                self.get_logger().info(
                    f'MOVE: pos={self.current_pos[0]:.2f},{self.current_pos[1]:.2f},{math.degrees(self.current_yaw):.1f} deg '
                    f'target={target[0]:.2f},{target[1]:.2f},{math.degrees(self.target_yaw):.1f} deg '
                    f'err={dx:.2f},{dy:.2f},{math.degrees(angle_error):.2f} deg '
                    f'vel=0.000,0.000,{vel_angle:.3f} (angle not ready)'
                )
                return
                
            self._publish_cmd(vel_forward, vel_side, vel_angle)

            self.get_logger().info(
                f'MOVE: angle_ready={self.angle_ready}, pos={self.current_pos[0]:.2f},{self.current_pos[1]:.2f},{math.degrees(self.current_yaw):.1f} deg '
                f'target={target[0]:.2f},{target[1]:.2f},{math.degrees(self.target_yaw):.1f} deg '
                f'err={dx:.2f},{dy:.2f},{math.degrees(angle_error):.2f} deg '
                f'vel={vel_forward:.3f},{vel_side:.3f},{vel_angle:.3f}'
            )
        elif self.state == self.ADJ:
            # 读取当前参数
            self._read_target_yaw()
            dx   = target[0] - self.current_pos[0]
            dy   = target[1] - self.current_pos[1]
            angle_error = self._normalize_angle(self.target_yaw - self.current_yaw)
            


            # 终止条件检测
            arrived = abs(dx) < self.arrive_precise_threshold and \
                      abs(dy) < self.arrive_precise_threshold and \
                      abs(angle_error) < self.angle_precise_threshold
            if arrived:
                self.get_logger().info(
                    f'ADJ: precisely aligned! idx={self.current_target_index}/'
                )
                if int(target[3]) != 0:
                    self.state = self.FETCH
                    self.get_logger().info(f'{len(self.current_path)},  entering FETCH')
                else:
                    if not self._move_to_next_target():
                        self._stop()
                        self.have_path = False
                        goal = SwitchNavigationState.Goal()
                        goal.target_state = 'state_b'

                        if not self.navigation_state_client.wait_for_server(timeout_sec=3.0):
                            self.get_logger().error('state_manager action server not available')
                            return

                        future = self.navigation_state_client.send_goal_async(goal)
                        self.get_logger().info('ADJ: all done, stopped')
                return  
            
            # 未到终点，进行修正，将位置修正与角度修正分开处理
            vel_forward,vel_side,vel_angle = 0,0,0
            if abs(angle_error) >= self.angle_precise_threshold and self.suspension_state in [-1,0,14,21]:
                vel_angle = self.pid_adj_angle.compute(angle_error, dt)
                if vel_angle > 0: vel_angle = max(0.15, min(self._p('max_adj_angular'), vel_angle))
                elif vel_angle < 0: vel_angle = min(-0.15, max(-self._p('max_adj_angular'), vel_angle))

            # 设置速度
            # - 计算距离
            if(abs(math.degrees(self.target_yaw))<1e-6): dis_forward,dis_side = dx, dy
            elif(abs(math.degrees(self.target_yaw)-90)<1e-6): dis_forward,dis_side = dy, -dx
            elif(abs(math.degrees(self.target_yaw)+90)<1e-6): dis_forward,dis_side = -dy, dx
            else: dis_forward,dis_side = -dx, -dy

            # - 轴向
            vel_forward = self.pid_adj_dist.compute(dis_forward, dt)
            if abs(dis_forward) >= self.arrive_precise_threshold:
                if vel_forward > 0: vel_forward = max(0.15, min(self._p('max_adj_vel'), vel_forward))
                elif vel_forward < 0: vel_forward = min(-0.15, max(-self._p('max_adj_vel'), vel_forward))
            else: vel_forward = 0.0
            
            # - 侧向
            vel_side = self.pid_adj_side.compute(dis_side, dt)
            if abs(dis_side) >= self.arrive_precise_threshold:
                if vel_side > 0: vel_side = max(0.15, min(self._p('max_adj_vel'), vel_side))
                elif vel_side < 0: vel_side = min(-0.15, max(-self._p('max_adj_vel'), vel_side))
            else: vel_side = 0.0
                
            self._publish_cmd(vel_forward, vel_side, vel_angle)

            self.get_logger().info(
                f'ADJ: angle_ready={self.angle_ready}, pos={self.current_pos[0]:.2f},{self.current_pos[1]:.2f},{math.degrees(self.current_yaw):.1f} deg '
                f'target={target[0]:.2f},{target[1]:.2f},{math.degrees(self.target_yaw):.1f} deg '
                f'err={dx:.2f},{dy:.2f},{math.degrees(angle_error):.2f} deg '
                f'vel={vel_forward:.3f},{vel_side:.3f},{vel_angle:.3f}'
            )
        elif self.state == self.FETCH:
            
            # 读取当前参数
            self._read_target_yaw()
            dx   = target[0] - self.current_pos[0] +2*self.reserved_length*math.cos(self.target_yaw)
            dy   = target[1] - self.current_pos[1] +2*self.reserved_length*math.sin(self.target_yaw)
            angle_error = self._normalize_angle(self.target_yaw - self.current_yaw)
            
            # 终止条件检测
            arrived = abs(dx) < self.arrive_precise_threshold and \
                      abs(dy) < self.arrive_precise_threshold
            if arrived:
                self.get_logger().info(
                    f'FETCH: can_do={target[3]} sent! idx={self.current_target_index}/'
                    f'{len(self.current_path)}, entering RET, waiting for can_go=True to return'
                )
                self.state = self.RET
                self.can_go = False
                return         

            self.suspension_state_pub.publish(Int32(data=-1))
            if target[3] == 200: self.suspension_control_pub.publish(Float32MultiArray(data=[175.0, 175.0, 175.0, 175.0]))
            elif target[3] == 400: self.suspension_control_pub.publish(Float32MultiArray(data=[375.0, 375.0, 375.0, 375.0]))

            # 未到终点，进行修正，将位置修正与角度修正分开处理
            vel_forward,vel_side,vel_angle = 0,0,0
            
            # 设置速度
            # - 计算距离
            if(abs(math.degrees(self.target_yaw))<1e-6): dis_forward,dis_side = dx, dy
            elif(abs(math.degrees(self.target_yaw)-90)<1e-6): dis_forward,dis_side = dy, -dx
            elif(abs(math.degrees(self.target_yaw)+90)<1e-6): dis_forward,dis_side = -dy, dx
            else: dis_forward,dis_side = -dx, -dy

            # - 轴向
            vel_forward = self.pid_adj_dist.compute(dis_forward, dt)
            if abs(dis_forward) >= self.arrive_precise_threshold:
                if vel_forward > 0: vel_forward = max(0.15, min(self._p('max_adj_vel'), vel_forward))
                elif vel_forward < 0: vel_forward = min(-0.15, max(-self._p('max_adj_vel'), vel_forward))
            else: vel_forward = 0.0
            
            # - 侧向
            vel_side = self.pid_adj_side.compute(dis_side, dt)
            if abs(dis_side) >= self.arrive_precise_threshold:
                if vel_side > 0: vel_side = max(0.15, min(self._p('max_adj_vel'), vel_side))
                elif vel_side < 0: vel_side = min(-0.15, max(-self._p('max_adj_vel'), vel_side))
            else: vel_side = 0.0
            self.get_logger().info(
                f'FETCH: angle_ready={self.angle_ready}, pos={self.current_pos[0]:.2f},{self.current_pos[1]:.2f},{math.degrees(self.current_yaw):.1f} deg '
                f'target={target[0]:.2f},{target[1]:.2f},{math.degrees(self.target_yaw):.1f} deg '
                f'err={dx:.2f},{dy:.2f},{math.degrees(angle_error):.2f} deg '
                f'vel={vel_forward:.3f},{vel_side:.3f},{vel_angle:.3f}'
            )
            self._publish_cmd(vel_forward, vel_side, 0)
        
        elif self.state == self.RET:
            if not self.can_go:
                self.can_do_pub.publish(Float32MultiArray(data=[1.0,float(target[3]),0.0,0.0,0.0,0.0,0.0,0.0]))
                if target[3] == 200: self.suspension_control_pub.publish(Float32MultiArray(data=[175.0, 175.0, 175.0, 175.0]))
                elif target[3] == 400: self.suspension_control_pub.publish(Float32MultiArray(data=[375.0, 375.0, 375.0, 375.0]))
                self._publish_cmd(0, 0, 0)
                return
            # 读取当前参数
            self._read_target_yaw()
            dx   = target[0] - self.current_pos[0]
            dy   = target[1] - self.current_pos[1]
            angle_error = self._normalize_angle(self.target_yaw - self.current_yaw)
            
            # 终止条件检测
            arrived = abs(dx) < self.arrive_precise_threshold and \
                      abs(dy) < self.arrive_precise_threshold
            if arrived:
                self.suspension_control_pub.publish(Float32MultiArray(data=[30.0, 30.0, 30.0, 30.0]))
                self.suspension_state_pub.publish(Int32(data=0))
                self.get_logger().info(
                    f'RET: returned! idx={self.current_target_index}/'
                    f'{len(self.current_path)}, calling _move_to_next_target '
                )
                if not self._move_to_next_target():
                    self._stop()
                    self.have_path = False
                    goal = SwitchNavigationState.Goal()
                    goal.target_state = 'state_b'

                    if not self.navigation_state_client.wait_for_server(timeout_sec=3.0):
                        self.get_logger().error('state_manager action server not available')
                        return

                    future = self.navigation_state_client.send_goal_async(goal)
                    self.get_logger().info('RET: all done, stopped')
                return         


            self.suspension_state_pub.publish(Int32(data=-1))
            if target[3] == 200: self.suspension_control_pub.publish(Float32MultiArray(data=[175.0, 175.0, 175.0, 175.0]))
            elif target[3] == 400: self.suspension_control_pub.publish(Float32MultiArray(data=[375.0, 375.0, 375.0, 375.0]))
            
            # 未到终点，进行修正，将位置修正与角度修正分开处理
            vel_forward,vel_side,vel_angle = 0,0,0
            
            # 设置速度
            # - 计算距离
            if(abs(math.degrees(self.target_yaw))<1e-6): dis_forward,dis_side = dx, dy
            elif(abs(math.degrees(self.target_yaw)-90)<1e-6): dis_forward,dis_side = dy, -dx
            elif(abs(math.degrees(self.target_yaw)+90)<1e-6): dis_forward,dis_side = -dy, dx
            else: dis_forward,dis_side = -dx, -dy

            # - 轴向
            vel_forward = self.pid_adj_dist.compute(dis_forward, dt)
            if abs(dis_forward) >= self.arrive_precise_threshold:
                if vel_forward > 0: vel_forward = max(0.15, min(self._p('max_adj_vel'), vel_forward))
                elif vel_forward < 0: vel_forward = min(-0.15, max(-self._p('max_adj_vel'), vel_forward))
            else: vel_forward = 0.0
            
            # - 侧向
            vel_side = self.pid_adj_side.compute(dis_side, dt)
            if abs(dis_side) >= self.arrive_precise_threshold:
                if vel_side > 0: vel_side = max(0.15, min(self._p('max_adj_vel'), vel_side))
                elif vel_side < 0: vel_side = min(-0.15, max(-self._p('max_adj_vel'), vel_side))
            else: vel_side = 0.0
            self.get_logger().info(
                f'RET: angle_ready={self.angle_ready}, pos={self.current_pos[0]:.2f},{self.current_pos[1]:.2f},{math.degrees(self.current_yaw):.1f} deg '
                f'target={target[0]:.2f},{target[1]:.2f},{math.degrees(self.target_yaw):.1f} deg '
                f'err={dx:.2f},{dy:.2f},{math.degrees(angle_error):.2f} deg '
                f'vel={vel_forward:.3f},{vel_side:.3f},{vel_angle:.3f}'
            )
            self._publish_cmd(vel_forward, vel_side, 0)
            
            
    # =======================================================================
    #  全向模式
    # =======================================================================
    def _control_omni(self, target):
            
        dt = 1.0 / self.control_frequency
        
        if self.state == self.HOLD:
            
            self._stop()
            
            if self.can_go:
                self.state = self.MOVE
                self._reset_pids()
                self.get_logger().info('HOLD: can_go=True, entering MOVE')
            return

        elif self.state == self.MOVE:
            
            # 读取当前参数
            self._read_target_yaw()
            dx   = target[0] - self.current_pos[0]
            dy   = target[1] - self.current_pos[1]
            angle_error = self._normalize_angle(-self.current_yaw)

            # 终止条件检测
            arrived = abs(dx) < self.arrive_threshold and \
                      abs(dy) < self.arrive_threshold and \
                      abs(angle_error) < self.angle_threshold
            if arrived:
                self.get_logger().info(
                    f'MOVE: roughly arrived! idx={self.current_target_index}/'
                    f'{len(self.current_path)}, entering ADJ to precisely align.'
                )
                self.state = self.ADJ
                return            
            
            # 未到终点，进行修正，将位置修正与角度修正分开处理
            vel_x,vel_y,vel_angle = 0,0,0
            if abs(angle_error) >= self.angle_threshold and self.suspension_state in [-1,0,14,21]:
                vel_angle = self.pid_angle.compute(angle_error, dt)
                if vel_angle > 0: vel_angle = max(0.15, min(self._p('max_angular'), vel_angle))
                elif vel_angle < 0: vel_angle = min(-0.15, max(-self._p('max_angular'), vel_angle))
                
            # 设置速度
            # - x方向
            vel_x = self.pid_dist.compute(dx, dt)
            if abs(dx) >= self.arrive_threshold:
                if vel_x > 0: vel_x = max(0.15, min(self._p('max_vel'), vel_x))
                elif vel_x < 0: vel_x = min(-0.15, max(-self._p('max_vel'), vel_x))
            else: vel_x = 0.0
                
            # - y方向
            vel_y = self.pid_side.compute(dy, dt)
            if abs(dy) >= self.arrive_threshold:
                if vel_y > 0: vel_y = max(0.15, min(self._p('max_vel'), vel_y))
                elif vel_y < 0: vel_y = min(-0.15, max(-self._p('max_vel'), vel_y))
            else: vel_y = 0.0
                
            self._publish_cmd(vel_x, vel_y, vel_angle)

            self.get_logger().info(
                f'MOVE: pos={self.current_pos[0]:.2f},{self.current_pos[1]:.2f},{math.degrees(self.current_yaw):.1f} deg '
                f'target={target[0]:.2f},{target[1]:.2f},{math.degrees(self.target_yaw):.1f} deg '
                f'err={dx:.2f},{dy:.2f},{math.degrees(angle_error):.2f} deg '
                f'vel={vel_x:.3f},{vel_y:.3f},{vel_angle:.3f}'
            )
        elif self.state == self.ADJ:
            
            # 读取当前参数
            self._read_target_yaw()
            dx   = target[0] - self.current_pos[0]
            dy   = target[1] - self.current_pos[1]
            angle_error = self._normalize_angle(-self.current_yaw)

            # 终止条件检测
            arrived = abs(dx) < self.arrive_precise_threshold and \
                      abs(dy) < self.arrive_precise_threshold and \
                      abs(angle_error) < self.angle_precise_threshold
            if arrived:
                self.get_logger().info(
                    f'ADJ: precisely arrived! idx={self.current_target_index}/'
                    f'{len(self.current_path)}, calling _move_to_next_target'
                )
                if not self._move_to_next_target():
                    self._stop()
                    self.have_path = False
                    self.get_logger().info('ADJ: all done, stopped')
                return            
            
            # 未到终点，进行修正，将位置修正与角度修正分开处理
            vel_x,vel_y,vel_angle = 0,0,0
            if abs(angle_error) >= self.angle_precise_threshold and self.suspension_state in [-1,0,14,21]:
                vel_angle = self.pid_adj_angle.compute(angle_error, dt)
                if vel_angle > 0: vel_angle = max(0.15, min(self._p('max_adj_angular'), vel_angle))
                elif vel_angle < 0: vel_angle = min(-0.15, max(-self._p('max_adj_angular'), vel_angle))
                
            # 设置速度
            # - x方向
            vel_x = self.pid_adj_dist.compute(dx, dt)
            if abs(dx) >= self.arrive_precise_threshold:
                if vel_x > 0: vel_x = max(0.15, min(self._p('max_adj_vel'), vel_x))
                elif vel_x < 0: vel_x = min(-0.15, max(-self._p('max_adj_vel'), vel_x))
            else: vel_x = 0.0
                
            # - y方向
            vel_y = self.pid_adj_side.compute(dy, dt)
            if abs(dy) >= self.arrive_precise_threshold:
                if vel_y > 0: vel_y = max(0.15, min(self._p('max_adj_vel'), vel_y))
                elif vel_y < 0: vel_y = min(-0.15, max(-self._p('max_adj_vel'), vel_y))
            else: vel_y = 0.0
                
            self._publish_cmd(vel_x, vel_y, vel_angle)

            self.get_logger().info(
                f'ADJ: pos={self.current_pos[0]:.2f},{self.current_pos[1]:.2f},{math.degrees(self.current_yaw):.1f} deg '
                f'target={target[0]:.2f},{target[1]:.2f},{math.degrees(self.target_yaw):.1f} deg '
                f'err={dx:.2f},{dy:.2f},{math.degrees(angle_error):.2f} deg '
                f'vel={vel_x:.3f},{vel_y:.3f},{vel_angle:.3f}'
            )

    def _stop(self):
        self._publish_cmd(0.0, 0.0, 0.0)

    def _publish_cmd(self, vx, vy, vyaw):
        msg = Float32MultiArray()
        msg.data = [float(vx), float(vy), float(vyaw)]
        self.cmd_pub.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
