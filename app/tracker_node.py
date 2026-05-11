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

    TURN, MOVE, HOLD = 0, 1, 2

    def __init__(self):
        super().__init__('tracker')

        # --- 运动模式 ---
        self.declare_parameter('motion_type',        'unidirectional')
        self.declare_parameter('control_frequency',  50.0)
        self.declare_parameter('arrive_threshold',    0.12)

        # --- Topics ---
        self.declare_parameter('planning_path',       '/planning/path')
        self.declare_parameter('odom_world',         '/odom_world')
        self.declare_parameter('can_go',            '/can_go')
        self.declare_parameter('cmd_vel',            '/t0x0101')
        self.declare_parameter('tracker_direction',  '/direction')
        self.declare_parameter('target_yaw_deg',      '/target_yaw_deg')

        # --- 全向模式 PID ---
        self.declare_parameter('kp_x',   6.0); self.declare_parameter('ki_x',   0.0); self.declare_parameter('kd_x',   0.2)
        self.declare_parameter('kp_y',   6.0); self.declare_parameter('ki_y',   0.0); self.declare_parameter('kd_y',   0.2)
        self.declare_parameter('kp_yaw', 2.0); self.declare_parameter('ki_yaw', 0.0); self.declare_parameter('kd_yaw', 0.5)
        self.declare_parameter('max_vel_x',  0.5)
        self.declare_parameter('max_vel_y',  0.5)
        self.declare_parameter('max_vel_yaw', 2.0)

        # --- 单向模式 PID ---
        self.declare_parameter('kp_angle',  4.0); self.declare_parameter('ki_angle',  0.0); self.declare_parameter('kd_angle',  0.2)
        self.declare_parameter('kp_dist',   2.5); self.declare_parameter('ki_dist',   0.0); self.declare_parameter('kd_dist',   0.1)
        self.declare_parameter('max_vel_forward', 0.5)
        self.declare_parameter('max_angular',      2.0)

        # --- 直线闭环校正 PID ---
        self.declare_parameter('kp_lateral',       2.5)
        self.declare_parameter('ki_lateral',       0.0)
        self.declare_parameter('kd_lateral',       0.1)
        self.declare_parameter('max_correction',   0.3)
        self.declare_parameter('lateral_threshold', 0.05)

        self.motion_type         = self.get_parameter('motion_type').value
        self.control_frequency   = self.get_parameter('control_frequency').value
        self.arrive_threshold    = self.get_parameter('arrive_threshold').value

        self.get_logger().info(f'Tracker Node started, motion_type={self.motion_type}')

        self.current_path          = None
        self.current_target_index  = 0
        self._move_first_iteration = False
        self.current_pos           = np.array([0.0, 0.0, 0.0])
        self.current_yaw           = 0.0
        self.target_yaw            = 0.0
        self.have_path             = False
        self.have_odom             = False
        self.can_go                = False
        self.prev_can_go           = False
        self.T = np.array([
        [-1.0, 0.0, 0.0, 0.35],
        [0.0,  -1.0, 0.0, 0.0],
        [0.0,  0.0, 1.0, 0.0],
        [0.0,  0.0, 0.0, 1.0]
        ])
        # 直线闭环状态变量
        self._move_start_pos = np.array([0.0, 0.0])

        self.state = self.TURN
        self.state_switched = False

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
        if self.motion_type == 'omnidirectional':
            self.pid_x   = PIDController(self._p('kp_x'),   self._p('ki_x'),   self._p('kd_x'),   self._p('max_vel_x'))
            self.pid_y   = PIDController(self._p('kp_y'),   self._p('ki_y'),   self._p('kd_y'),   self._p('max_vel_y'))
            self.pid_yaw = PIDController(self._p('kp_yaw'), self._p('ki_yaw'), self._p('kd_yaw'), self._p('max_vel_yaw'))
            self.pid_lat = PIDController(self._p('kp_lateral'), self._p('ki_lateral'), self._p('kd_lateral'), self._p('max_correction'))
        else:
            self.pid_angle = PIDController(self._p('kp_angle'), self._p('ki_angle'), self._p('kd_angle'), self._p('max_angular'))
            self.pid_dist  = PIDController(self._p('kp_dist'),  self._p('ki_dist'),  self._p('kd_dist'),  self._p('max_vel_forward'))
            self.pid_lat   = PIDController(self._p('kp_lateral'), self._p('ki_lateral'), self._p('kd_lateral'), self._p('max_correction'))

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

    def _init_publishers(self):
        self.cmd_pub = self.create_publisher(Float32MultiArray, self._p('cmd_vel', '/t0x0101'), 10)
        self.dir_pub = self.create_publisher(Int32,    self._p('tracker_direction',  '/direction'),         10)
        self.yaw_pub = self.create_publisher(Float32,  self._p('target_yaw_deg',    '/target_yaw_deg'),    10)

    def _path_callback(self, msg):
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return

        new_target_idx = 1
        if new_target_idx >= len(msg.poses):
            self.get_logger().warn('Path too short after skipping first point')
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

        self.current_path           = msg
        self.current_target_index   = new_target_idx
        self.have_path              = True
        self._move_first_iteration = True
        self.state                  = self.TURN
        self._reset_pids()
        self._read_target_yaw()
        self._publish_direction()
        self.yaw_pub.publish(Float32(data=math.degrees(self.target_yaw)))
        self.get_logger().info(f'Received path with {len(msg.poses)} points')

    def _odom_callback(self, msg):
        current_pose = [0, 0, 0, 0, 0, 0, 0]
        current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        transformer = PoseTransformerQuat()
        new_pose = transformer.apply_matrix_to_pose(current_pose, self.T)
        self.current_pos[0] = new_pose[0] - self.T[0][3]
        self.current_pos[1] = new_pose[1]
        self.current_pos[2] = new_pose[2]
        self.quaterion = new_pose[3:8]
        self.current_yaw = yaw_from_quaternion(self.quaterion) + np.pi
        # self.get_logger().warn(f"""
        # yaw:{self.current_yaw}
        # """)
        self.have_odom = True

    def _can_go_callback(self, msg):
        self.prev_can_go = self.can_go
        self.can_go = msg.data

    def _read_target_yaw(self):
        if self.current_path is None or self.current_target_index >= len(self.current_path.poses):
            return

        if self.motion_type == 'unidirectional':
            idx = self.current_target_index
            if idx + 1 < len(self.current_path.poses):
                next_pose = self.current_path.poses[idx + 1].pose
                dx = next_pose.position.x - self.current_pos[0]
                dy = next_pose.position.y - self.current_pos[1]
                if math.hypot(dx, dy) < 1.0:
                    self.target_yaw = yaw_from_quaternion(next_pose.orientation)
                    return
            self.target_yaw = yaw_from_quaternion(
                self.current_path.poses[idx].pose.orientation
            )
        else:
            self.target_yaw = yaw_from_quaternion(
                self.current_path.poses[self.current_target_index].pose.orientation
            )

    def _reset_pids(self):
        if self.motion_type == 'omnidirectional':
            self.pid_x.reset(); self.pid_y.reset(); self.pid_yaw.reset()
        else:
            self.pid_angle.reset(); self.pid_dist.reset()
        if hasattr(self, 'pid_lat'):
            self.pid_lat.reset()

    def _publish_direction(self):
        diff      = self._normalize_angle(self.target_yaw - self.current_yaw)
        diff_deg  = math.degrees(diff)
        if diff_deg > 2.0:
            direction = -1
        elif diff_deg < -2.0:
            direction = 1
        else:
            direction = 0
        self.dir_pub.publish(Int32(data=direction))
        self.get_logger().info(
            f'PUBLISH direction={direction} '
            f'target_yaw_deg={math.degrees(self.target_yaw):.2f} diff={diff_deg:.2f}'
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

    def _check_arrived(self, target):
        dx = target[0] - self.current_pos[0]
        dy = target[1] - self.current_pos[1]
        return math.sqrt(dx * dx + dy * dy) < self.arrive_threshold

    def _compute_lateral_error(self, target) -> tuple:
        """
        计算当前机器人到直线路径的侧向误差。

        直线路径: 从 _move_start_pos → target
        将当前位置 pos 投影到这条直线上，计算投影点 pos_proj，
        侧向误差 = 当前位置到投影点的距离（垂直于路线方向）

        Returns:
            lateral_err: float, 侧向误差大小 (m)
            lateral_sign: float, +1 或 -1，指示偏差方向（用于确定校正方向）
        """
        start = self._move_start_pos
        pos   = self.current_pos[:2]

        dx_path = target[0] - start[0]
        dy_path = target[1] - start[1]
        path_len_sq = dx_path * dx_path + dy_path * dy_path

        if path_len_sq < 1e-6:
            return 0.0, 0.0

        # 当前位置在路径上的投影参数 t（t=0 在起点，t=1 在终点）
        t = max(0.0, min(1.0,
            ((pos[0] - start[0]) * dx_path + (pos[1] - start[1]) * dy_path) / path_len_sq
        ))

        # 投影点坐标
        proj_x = start[0] + t * dx_path
        proj_y = start[1] + t * dy_path

        # 侧向误差向量（垂直于路径方向）
        err_x = pos[0] - proj_x
        err_y = pos[1] - proj_y
        lateral_err = math.sqrt(err_x * err_x + err_y * err_y)

        # 确定方向：取垂直于路径的单位向量，乘以误差方向
        path_len = math.sqrt(path_len_sq)
        # 路径法向量（90° 旋转）
        nx = -dy_path / path_len
        ny =  dx_path / path_len
        # 法向量点积误差向量确定方向符号
        lateral_sign = -1.0 if (err_x * nx + err_y * ny) >= 0.0 else 1.0

        return lateral_err * lateral_sign, lateral_sign

    def _move_to_next_target(self):
        self.current_target_index += 1
        self.can_go      = False
        self.prev_can_go = False
        self.get_logger().info(
            f'_move_to_next_target: new_idx={self.current_target_index} '
            f'total={len(self.current_path.poses)}, waiting for can_go'
        )
        if self.current_target_index >= len(self.current_path.poses):
            self.get_logger().info('All targets reached, stopping')
            return False
        self.target_yaw = yaw_from_quaternion(
            self.current_path.poses[self.current_target_index].pose.orientation
        )
        self._reset_pids()
        self.state = self.TURN
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

        if self.motion_type == 'omnidirectional':
            self._control_omni(target)
        else:
            self._control_uni(target)

    # =======================================================================
    #  单向模式
    # =======================================================================
    def _control_uni(self, target):
        dt = 1.0 / self.control_frequency
        angle_error = self._normalize_angle(self.target_yaw - self.current_yaw)

        if self.state == self.TURN:
            angular_vel = self.pid_angle.compute(angle_error, dt)
            angular_vel = max(-self._p('max_angular'), min(self._p('max_angular'), angular_vel))
            if abs(angular_vel) < 0.05:
                self.state = self.HOLD
                self.can_go = False
                self.prev_can_go = False
                self._reset_pids()
                self.get_logger().info('Angle aligned, entering HOLD')
                return
            self._publish_cmd(0.0, 0.0, angular_vel)
            return

        elif self.state == self.HOLD:
            self._stop()
            if self.can_go:
                self.state = self.MOVE
                self._move_first_iteration = True
                self._reset_pids()
                self._move_start_pos = self.current_pos[:2].copy()
                self.get_logger().info('can_go=True, entering MOVE')
            return

        elif self.state == self.MOVE:
            skip_arrival_check = self._move_first_iteration
            self._move_first_iteration = False

            arrived = self._check_arrived(target)
            if not skip_arrival_check and arrived:
                self.get_logger().info(
                    f'MOVE: arrived! idx={self.current_target_index}/'
                    f'{len(self.current_path.poses)}, calling _move_to_next_target'
                )
                if not self._move_to_next_target():
                    self._stop()
                    self.have_path = False
                    self.get_logger().info('MOVE: all done, stopped')
                return

            self._read_target_yaw()
            dx   = target[0] - self.current_pos[0]
            dy   = target[1] - self.current_pos[1]
            
            # 计算目标点在车体前向轴（局部 X 轴）上的投影投影距离（带正负号）
            local_dx = dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)
            dist_abs = math.hypot(dx, dy)

            # ---- 前进速度 (改为使用本地纵向误差投影来计算，支持后退修正) ----
            forward_vel = self.pid_dist.compute(local_dx, dt)
            if dist_abs >= self.arrive_threshold:
                # 速度限幅，保留正负号
                sign = 1.0 if forward_vel > 0 else -1.0
                forward_vel = sign * max(0.15, min(self._p('max_vel_forward'), abs(forward_vel)))
            else:
                forward_vel = 0.0

            # ---- 角度 PID ----
            # 对于单向模式而言，不应该盲目锁死 target_yaw。如果距离较远，车头应当对准目标点。
            if dist_abs > 0.2:
                desired_yaw = math.atan2(dy, dx)
                # 倒车情况判定
                if local_dx < 0:
                    desired_yaw = self._normalize_angle(desired_yaw + math.pi)
            else:
                # 接近目标时，逐渐将姿态向路径终点姿态对齐
                desired_yaw = self.target_yaw

            angle_error = self._normalize_angle(desired_yaw - self.current_yaw)
            angular_vel = self.pid_angle.compute(angle_error, dt)
            angular_vel = max(-self._p('max_angular'), min(self._p('max_angular'), angular_vel))

            self.get_logger().info(
                f'MOVE: pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f}) '
                f'target=({target[0]:.2f},{target[1]:.2f}) '
                f'dist={dist_abs:.3f} local_dx={local_dx:.3f} angle_err={abs(angle_error):.3f}'
            )

            self._publish_cmd(forward_vel, 0.0, angular_vel)

    # =======================================================================
    #  全向模式
    # =======================================================================
    def _control_omni(self, target):
        dt = 1.0 / self.control_frequency
        angle_error = self._normalize_angle(self.target_yaw - self.current_yaw)

        if self.state == self.TURN:
            vyaw = self.pid_yaw.compute(angle_error, dt)
            vyaw = max(-self._p('max_vel_yaw'), min(self._p('max_vel_yaw'), vyaw))
            if abs(vyaw) < 0.05:
                self.state = self.HOLD
                self.can_go = False
                self.prev_can_go = False
                self._reset_pids()
                self.get_logger().info(f"""
                        Angle aligned, entering HOLD
                        current_yaw:{self.current_yaw}
                """)
                return
            self._publish_cmd(0.0, 0.0, vyaw)
            return

        elif self.state == self.HOLD:
            self._stop()
            if self.can_go:
                self.state = self.MOVE
                self._move_first_iteration = True
                self._reset_pids()
                self._move_start_pos = self.current_pos[:2].copy()
                self.get_logger().info('can_go=True, entering MOVE')
            return

        elif self.state == self.MOVE:
            skip_arrival_check = self._move_first_iteration
            self._move_first_iteration = False

            self._read_target_yaw()
            angle_error = self._normalize_angle(self.target_yaw - self.current_yaw)
            angle_err = abs(angle_error)

            dx   = target[0] - self.current_pos[0]
            dy   = target[1] - self.current_pos[1]
            dist = math.sqrt(dx * dx + dy * dy)

            if not skip_arrival_check and dist < self.arrive_threshold:
                if not self._move_to_next_target():
                    self._stop()
                    self.have_path = False
                    self.get_logger().info('MOVE: all done, stopped')
                return

            self.get_logger().info(
                f'MOVE: pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f}) '
                f'target=({target[0]:.2f},{target[1]:.2f}) '
                f'dist={dist:.3f} angle_err={angle_err:.3f}'
            )

            # --- 直线闭环校正：全向模式直接输出侧向速度 vy ---
            lateral_err, lat_sign = self._compute_lateral_error(target)
            lat_thresh  = self._p('lateral_threshold', 0.05)
            vy_correct  = 0.0
            if abs(lateral_err) > lat_thresh:
                vy_correct = self.pid_lat.compute(abs(lateral_err), dt)
                vy_correct = max(-self._p('max_correction'), min(self._p('max_correction'), vy_correct))
                vy_correct *= lat_sign
                self.get_logger().info(
                    f'  [line_correction] lateral_err={lateral_err:.3f} '
                    f'sign={lat_sign:.0f} vy_correct={vy_correct:.3f}'
                )

            # 姿态闭环校正：持续修正朝向
            vyaw_correction = self.pid_yaw.compute(angle_error, dt)
            vyaw_correction = max(-self._p('max_vel_yaw'), min(self._p('max_vel_yaw'), vyaw_correction))

            # 主方向速度（沿路径方向）
            progress = min(1.0, max(0.0, (dist - (self.arrive_threshold - 0.05)) / 0.05))
            vx = (dx / dist) * self._p('max_vel_x') * progress if dist > 1e-6 else 0.0
            vy = (dy / dist) * self._p('max_vel_y') * progress if dist > 1e-6 else 0.0

            # 叠加侧向校正速度
            vy += vy_correct

            vx = max(-self._p('max_vel_x'), min(self._p('max_vel_x'), vx))
            vy = max(-self._p('max_vel_y'), min(self._p('max_vel_y'), vy))
            self.get_logger().error(
                f"""
            current_yaw:{self.current_yaw}
            vx:{vx}
            vy:{vy}
            vyaw:{vyaw_correction}
            """
            )
            self._publish_cmd(vx, vy, vyaw_correction)

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
