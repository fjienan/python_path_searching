#!/usr/bin/env python3
"""
轨迹跟踪节点（逐段 PID 控制 + 直线闭环校正）

状态机流程：
  单向模式：TURN → HOLD → MOVE → TURN → ...
           - TURN: 转到目标角度（can_go 不影响）
           - HOLD: 停在关键点，等 can_go=True 才继续
           - MOVE: 直线前进到目标点，实时校正偏离直线误差

  全向模式：TURN → HOLD → MOVE → TURN → ...
           - TURN: 转到目标角度，转向与平移完全分开
           - HOLD: 停在关键点，等 can_go=True 才继续
           - MOVE: 直线移动到目标点，vy 侧向 PID 校正

直线闭环原理：
  记录进入 MOVE 时的起点 pos_start 和目标 target，计算理想路径方向向量 d_path。
  将当前位置 pos 投影到这条直线上得到 pos_proj，计算侧向误差 lateral_err = |pos - pos_proj|_⊥。
  侧向误差超过 lateral_threshold 时，施加 PID 校正力将机器人拉回直线。
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, Int32, Float32, Float32MultiArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import math
import numpy as np

from core.pid_controller import PIDController
from robot_pose.transformer import PoseTransformerQuat
from core.transform_utils import yaw_from_quaternion


class TrackerNode(Node):

    MOVE, HOLD = 1, 2

    def __init__(self):
        super().__init__('tracker')
        
        # --- 地图参数 ---
        self.declare_parameter('map_origin',        [3.2, 1.2, 0.0])

        # --- 运动模式 ---
        self.declare_parameter('motion_type',        'unidirectional')
        self.declare_parameter('control_frequency',  50.0)
        self.declare_parameter('arrive_threshold',    0.12)
        self.declare_parameter('angle_threshold',    0.08)

        # --- Topics ---
        self.declare_parameter('planning_path',       '/planning/path')
        self.declare_parameter('odom_world',         '/odom_world')
        self.declare_parameter('can_go',            '/can_go')
        self.declare_parameter('cmd_vel',            '/cmd_vel')
        self.declare_parameter('tracker_direction',  '/direction')
        self.declare_parameter('target_yaw_deg',      '/target_yaw_deg')
        self.declare_parameter('suspension_state', '/current_state')

        # --- PID ---
        self.declare_parameter('kp_angle', 4.0); self.declare_parameter('ki_angle', 0.0); self.declare_parameter('kd_angle', 0.2)
        self.declare_parameter('kp_dist',  2.5); self.declare_parameter('ki_dist',  0.0); self.declare_parameter('kd_dist',  0.1)
        self.declare_parameter('max_vel', 0.5)
        self.declare_parameter('max_angular', 2.0)


        self.map_origin          = tuple(self._p('map_origin'))
        self.motion_type         = self._p('motion_type')
        self.control_frequency   = self._p('control_frequency')
        self.arrive_threshold    = self._p('arrive_threshold')
        self.angle_threshold     = self._p('angle_threshold')

        self.get_logger().info(f'__init__: Tracker Node started, motion_type={self.motion_type}')

        self.current_path          = None
        self.current_target_index  = 0
        self.current_pos           = np.array([0.0, 0.0, 0.0])
        self.current_yaw           = 0.0
        self.target_yaw            = 0.0
        self.have_path             = False
        self.have_odom             = False
        self.can_go                = False
        self.T = np.array([
        [-1.0, 0.0, 0.0, 0.38],
        [0.0,  -1.0, 0.0, 0.0],
        [0.0,  0.0, 1.0, 0.0],
        [0.0,  0.0, 0.0, 1.0]
        ])
        # 直线闭环状态变量
        self._move_start_pos = np.array([0.0, 0.0])

        self.state = self.HOLD
        self.state_switched = False
        
        self.suspension_state = 0

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

    def _init_subscribers(self):
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_sub   = self.create_subscription(Path,      self._p('planning_path', '/planning/path'), self._path_callback,  qos)
        self.odom_sub   = self.create_subscription(Odometry, self._p('odom_world',   '/odom_world'),   self._odom_callback,   10)
        self.can_go_sub = self.create_subscription(Bool,     self._p('can_go',       '/can_go'),       self._can_go_callback, 10)
        self.suspension_state = self.create_subscription(Int32, self._p('suspension_state', '/current_state'), self._suspension_state_callback, 10)

    def _init_publishers(self):
        self.cmd_pub = self.create_publisher(Float32MultiArray, self._p('cmd_vel', '/t0x0101'), 10)
        self.dir_pub = self.create_publisher(Int32,    self._p('tracker_direction',  '/direction'),         10)
        self.yaw_pub = self.create_publisher(Float32,  self._p('target_yaw_deg',    '/target_yaw_deg'),    10)

    def _suspension_state_callback(self, msg):
        self.suspension_state = msg.data

    def _path_callback(self, msg):
        if len(msg.poses) == 0:
            self.get_logger().warn('_path_callback: Received empty path')
            return

        new_target_idx = 1
        if new_target_idx >= len(msg.poses):
            self.get_logger().warn('_path_callback: Path too short after skipping first point')
            return

        if self.current_path is not None and self.have_path:
            cur_target = self._get_current_target()
            if cur_target is not None:
                new_target = np.array([
                    msg.poses[new_target_idx].pose.position.x,
                    msg.poses[new_target_idx].pose.position.y
                ])
                if np.allclose(cur_target, new_target, atol=0.05):
                    self.current_path = msg
                    return

        self.current_path = msg
        self.get_logger().info(f'_path_callback: Received path with {len(msg.poses)} points')
        for i in range(len(self.current_path.poses)):
            ox = self.current_path.poses[i].pose.position.x
            oy = self.current_path.poses[i].pose.position.y
            ori = self._normalize_angle(yaw_from_quaternion(self.current_path.poses[i].pose.orientation))
            self.get_logger().info(f'_path_callback: Path point {i}: x={ox:.3f}, y={oy:.3f}, yaw={math.degrees(ori)} deg')

        self.current_target_index   = new_target_idx
        self.have_path              = True
        self.state                  = self.HOLD
        self._reset_pids()
        self._read_target_yaw()
        self._publish_direction()
        self.yaw_pub.publish(Float32(data=math.degrees(self.target_yaw)))
        self.get_logger().info(f'_path_callback: new_idx={self.current_target_index} '
            f'total={len(self.current_path.poses)}, entering HOLD')

    def _odom_callback_with_transform(self, msg):
        current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, 
                        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        transformer = PoseTransformerQuat()
        new_pose = transformer.apply_matrix_to_pose(current_pose, self.T)
        self.current_pos[0] = new_pose[0]
        self.current_pos[1] = new_pose[1]
        self.current_pos[2] = new_pose[2]
        self.quaterion = new_pose[3:8]
        self.current_yaw = self._normalize_angle(yaw_from_quaternion(self.quaterion) + np.pi)
        self.have_odom = True
        
    def _odom_callback(self, msg):
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z
        self.quaterion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        self.current_yaw = self._normalize_angle(yaw_from_quaternion(self.quaterion))
        self.have_odom = True

    def _can_go_callback(self, msg):
        self.can_go = msg.data

    def _read_target_yaw(self):
        if self.current_path is None or self.current_target_index >= len(self.current_path.poses): return
        self.target_yaw = self._normalize_angle(yaw_from_quaternion(self.current_path.poses[self.current_target_index].pose.orientation))

    def _reset_pids(self):
        self.pid_angle.reset(); self.pid_dist.reset()

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
        if self.current_path is None or self.current_target_index >= len(self.current_path.poses):
            return None
        pose = self.current_path.poses[self.current_target_index].pose
        return np.array([pose.position.x, pose.position.y])

    def _move_to_next_target(self):
        self.current_target_index += 1
        self.can_go = False
        self.get_logger().info(
            f'_move_to_next_target: new_idx={self.current_target_index} '
            f'total={len(self.current_path.poses)}, entering HOLD'
        )
        if self.current_target_index >= len(self.current_path.poses):
            self.get_logger().info('_move_to_next_target: All targets reached, stopping')
            return False
        self._read_target_yaw()
        self._reset_pids()
        self.state=self.HOLD
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
            angle_error = self._normalize_angle(self.target_yaw - self.current_yaw)
            
            # 计算轴向和侧向所需距离
            theta = self.current_yaw-math.atan2(1.0*dy,1.0*dx)
            dis_forward = math.hypot(dy,dx)*math.cos(theta)
            dis_side = -math.hypot(dy,dx)*math.sin(theta)

            # 终止条件检测
            arrived = abs(dis_forward) < self.arrive_threshold and \
                      abs(dis_side) < self.arrive_threshold and \
                      abs(angle_error) < self.angle_threshold
            if arrived:
                self.get_logger().info(
                    f'MOVE: arrived! idx={self.current_target_index}/'
                    f'{len(self.current_path.poses)}, calling _move_to_next_target'
                )
                if not self._move_to_next_target():
                    self._stop()
                    self.have_path = False
                    self.get_logger().info('MOVE: all done, stopped')
                return            
            
            # 未到终点，进行修正，将位置修正与角度修正分开处理
            vel_forward,vel_side,vel_angle = 0,0,0
            if abs(angle_error) >= self.angle_threshold and self.suspension_state in [0,14,21]:
                vel_angle = self.pid_angle.compute(angle_error, dt)
                if vel_angle > 0: vel_angle = max(0.15, min(self._p('max_angular'), vel_angle))
                elif vel_angle < 0: vel_angle = min(-0.15, max(-self._p('max_angular'), vel_angle))
                self._publish_cmd(0, 0, vel_angle)
            else:
                # 设置速度
                # - 轴向
                vel_forward = self.pid_dist.compute(dis_forward, dt)
                if abs(dis_forward) >= self.arrive_threshold:
                    if vel_forward > 0: vel_forward = max(0.15, min(self._p('max_vel'), vel_forward))
                    elif vel_forward < 0: vel_forward = min(-0.15, max(-self._p('max_vel'), vel_forward))
                else: vel_forward = 0.0
                
                # - 侧向
                vel_side = self.pid_dist.compute(dis_side, dt)
                if abs(dis_side) >= self.arrive_threshold:
                    if vel_side > 0: vel_side = max(0.15, min(self._p('max_vel'), vel_side))
                    elif vel_side < 0: vel_side = min(-0.15, max(-self._p('max_vel'), vel_side))
                else: vel_side = 0.0
                
                self._publish_cmd(vel_forward, vel_side, 0)

            self.get_logger().info(
                f'MOVE: pos={self.current_pos[0]:.2f},{self.current_pos[1]:.2f},{math.degrees(self.current_yaw):.1f} deg '
                f'target={target[0]:.2f},{target[1]:.2f},{math.degrees(self.target_yaw):.1f} deg '
                f'err={dx:.2f},{dy:.2f},{math.degrees(angle_error):.2f} deg '
                f'to_path={dis_forward:.2f},{dis_side:.2f},{math.degrees(theta):.2f} deg '
                f'vel={vel_forward:.3f},{vel_side:.3f},{vel_angle:.3f}'
            )

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
                    f'MOVE: arrived! idx={self.current_target_index}/'
                    f'{len(self.current_path.poses)}, calling _move_to_next_target'
                )
                if not self._move_to_next_target():
                    self._stop()
                    self.have_path = False
                    self.get_logger().info('MOVE: all done, stopped')
                return            
            
            # 未到终点，进行修正，将位置修正与角度修正分开处理
            vel_x,vel_y,vel_angle = 0,0,0
            if abs(angle_error) >= self.angle_threshold and self.suspension_state in [0,14,21]:
                vel_angle = self.pid_angle.compute(angle_error, dt)
                if vel_angle > 0: vel_angle = max(0.15, min(self._p('max_angular'), vel_angle))
                elif vel_angle < 0: vel_angle = min(-0.15, max(-self._p('max_angular'), vel_angle))
                self._publish_cmd(0, 0, vel_angle)
            else:
                # 设置速度
                # - x方向
                vel_x = self.pid_dist.compute(dx, dt)
                if abs(dx) >= self.arrive_threshold:
                    if vel_x > 0: vel_x = max(0.15, min(self._p('max_vel'), vel_x))
                    elif vel_x < 0: vel_x = min(-0.15, max(-self._p('max_vel'), vel_x))
                else: vel_x = 0.0
                
                # - y方向
                vel_y = self.pid_dist.compute(dy, dt)
                if abs(dy) >= self.arrive_threshold:
                    if vel_y > 0: vel_y = max(0.15, min(self._p('max_vel'), vel_y))
                    elif vel_y < 0: vel_y = min(-0.15, max(-self._p('max_vel'), vel_y))
                else: vel_y = 0.0
                
                self._publish_cmd(vel_x, vel_y, 0)

            self.get_logger().info(
                f'MOVE: pos={self.current_pos[0]:.2f},{self.current_pos[1]:.2f},{math.degrees(self.current_yaw):.1f} deg '
                f'target={target[0]:.2f},{target[1]:.2f},{math.degrees(self.target_yaw):.1f} deg '
                f'err={dx:.2f},{dy:.2f},{math.degrees(angle_error):.2f} deg '
                f'vel={vel_x:.3f},{vel_y:.3f},{vel_angle:.3f}'
            )

    def _stop(self):
        self._publish_cmd(0.0, 0.0, 0.0)




def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
