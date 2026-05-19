#!/usr/bin/env python3
"""
轨迹跟踪节点 — 状态机：HOLD → MOVE → ADJ → (HOLD depending on meta) → MOVE...

path_meta 编码 (Int32MultiArray, 与 Path 等长):
    0 = 普通移动点 (粗精度 ADJ, 跳过 HOLD)
    1 = KFS 抓取点   (精精度 ADJ, 进入 HOLD)
    2 = 起点/HOLD 点  (粗精度 ADJ, 进入 HOLD)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, Int32, Float32, Float32MultiArray, Int32MultiArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import math
import numpy as np

from core.pid_controller import PIDController
from core.transform_utils import yaw_from_quaternion


class TrackerNode(Node):

    ADJ, MOVE, HOLD = 0, 1, 2

    def __init__(self):
        super().__init__('tracker')

        self.declare_parameter('map_origin',        [3.2, 1.2, 0.0])
        self.declare_parameter('motion_type',        'unidirectional')
        self.declare_parameter('control_frequency',  50.0)
        self.declare_parameter('arrive_threshold',    0.12)
        self.declare_parameter('angle_threshold',    0.08)
        self.declare_parameter('arrive_precise_threshold',    0.01)
        self.declare_parameter('angle_precise_threshold',    0.02)
        self.declare_parameter('arrive_mid_threshold',       0.05)
        self.declare_parameter('angle_mid_threshold',        0.04)

        self.declare_parameter('planning_path',       '/planning/path')
        self.declare_parameter('planning_path_meta',  '/planning/path_meta')
        self.declare_parameter('odom_world',         '/odom_world')
        self.declare_parameter('can_go',            '/can_go')
        self.declare_parameter('cmd_vel',            '/cmd_vel')
        self.declare_parameter('tracker_direction',  '/direction')
        self.declare_parameter('target_yaw_deg',      '/target_yaw_deg')
        self.declare_parameter('suspension_state', '/current_state')

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
        self.arrive_mid_threshold = self._p('arrive_mid_threshold')
        self.angle_mid_threshold = self._p('angle_mid_threshold')

        self.get_logger().info(f'Tracker started, motion_type={self.motion_type}')

        self.current_path          = None
        self.current_path_meta     = []
        self.current_target_index  = 0
        self.current_pos           = np.array([0.0, 0.0, 0.0])
        self.current_yaw           = 0.0
        self.target_yaw            = 0.0
        self.have_path             = False
        self.have_odom             = False
        self.can_go                = False
        self.angle_ready           = False
        self.is_first_hold         = False
        self._move_start_pos = np.array([0.0, 0.0])

        self.state = self.HOLD
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
        self.pid_adj_angle = PIDController(self._p('kp_adj_angle'), self._p('ki_adj_angle'), self._p('kd_adj_angle'), self._p('max_adj_angular'))
        self.pid_adj_dist  = PIDController(self._p('kp_adj_dist'),  self._p('ki_adj_dist'),  self._p('kd_adj_dist'),  self._p('max_adj_vel'))

    def _init_subscribers(self):
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_sub      = self.create_subscription(Path,           self._p('planning_path', '/planning/path'), self._path_callback, qos)
        self.path_meta_sub = self.create_subscription(Int32MultiArray, self._p('planning_path_meta', '/planning/path_meta'), self._path_meta_callback, qos)
        self.odom_sub      = self.create_subscription(Odometry,       self._p('odom_world', '/odom_world'),   self._odom_callback, 10)
        self.can_go_sub    = self.create_subscription(Bool,           self._p('can_go', '/can_go'),           self._can_go_callback, 10)
        self.sus_sub       = self.create_subscription(Int32,          self._p('suspension_state', '/current_state'), self._suspension_state_callback, 10)

    def _init_publishers(self):
        self.cmd_pub = self.create_publisher(Float32MultiArray, self._p('cmd_vel', '/t0x0101'), 10)
        self.dir_pub = self.create_publisher(Int32,    self._p('tracker_direction', '/direction'), 10)
        self.yaw_pub = self.create_publisher(Float32,  self._p('target_yaw_deg', '/target_yaw_deg'), 10)

    def _suspension_state_callback(self, msg):
        self.suspension_state = msg.data

    def _path_meta_callback(self, msg):
        self.current_path_meta = msg.data
        self.get_logger().info(f'Received path_meta: {list(self.current_path_meta)}')

    def _path_callback(self, msg):
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return

        new_target_idx = 1
        if new_target_idx >= len(msg.poses):
            self.get_logger().warn('Path too short')
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
        self.get_logger().info(f'Received path with {len(msg.poses)} points')
        for i in range(len(self.current_path.poses)):
            ox = self.current_path.poses[i].pose.position.x
            oy = self.current_path.poses[i].pose.position.y
            ori = self._normalize_angle(yaw_from_quaternion(self.current_path.poses[i].pose.orientation))
            self.get_logger().info(f'  Point {i}: x={ox:.3f}, y={oy:.3f}, yaw={math.degrees(ori)} deg')

        self.current_target_index   = new_target_idx
        self.have_path              = True
        self.is_first_hold          = True
        self.state                  = self.HOLD
        self.angle_ready            = False
        self._reset_pids()
        self._read_target_yaw()
        self._publish_direction()
        self.yaw_pub.publish(Float32(data=math.degrees(self.target_yaw)))
        self.get_logger().info(f'New path: target_idx={self.current_target_index}, entering HOLD')

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
        self.pid_adj_angle.reset(); self.pid_adj_dist.reset()

    def _publish_direction(self):
        if self._p("motion_type") == "unidirectional":
            self.dir_pub.publish(Int32(data=0))
            return
        direction = 0
        tar_yaw_deg = math.degrees(self.target_yaw)
        if tar_yaw_deg > 2.0: direction = -1
        elif tar_yaw_deg < -2.0: direction = 1
        self.dir_pub.publish(Int32(data=direction))

    def _normalize_angle(self, angle):
        while angle > math.pi:  angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def _get_current_target(self):
        if self.current_path is None or self.current_target_index >= len(self.current_path.poses):
            return None
        pose = self.current_path.poses[self.current_target_index].pose
        return np.array([pose.position.x, pose.position.y])

    def _get_current_meta(self):
        idx = self.current_target_index
        if idx < len(self.current_path_meta):
            return self.current_path_meta[idx]
        return 0

    def _get_adj_thresholds(self):
        meta = self._get_current_meta()
        if meta == 1:
            return self.arrive_precise_threshold, self.angle_precise_threshold
        return self.arrive_mid_threshold, self.angle_mid_threshold

    def _advance_to_next_target(self, skip_hold=False):
        self.current_target_index += 1
        self.angle_ready = False
        self.get_logger().info(f'Advance: new_idx={self.current_target_index}/{len(self.current_path.poses)}')
        if self.current_target_index >= len(self.current_path.poses):
            self.get_logger().info('All targets reached, stopping')
            self._stop()
            self.have_path = False
            return False
        self._read_target_yaw()
        self._reset_pids()
        self._publish_direction()
        self.yaw_pub.publish(Float32(data=math.degrees(self.target_yaw)))
        if skip_hold:
            self.state = self.MOVE
            self.get_logger().info('Skipping HOLD, entering MOVE')
        else:
            self.state = self.HOLD
            self.can_go = False
            self.get_logger().info('Entering HOLD')
        return True

    def _move_to_next_target(self):
        return self._advance_to_next_target(skip_hold=False)

    # =======================================================================
    #  控制主循环
    # =======================================================================

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

        if self.state == self.HOLD:
            self._stop()
            meta = self._get_current_meta()
            if not self.is_first_hold and meta == 0:
                self._advance_to_next_target(skip_hold=True)
                return

            if self.can_go:
                self.is_first_hold = False
                self.state = self.MOVE
                self._reset_pids()
                self.get_logger().info(f'HOLD: can_go=True, meta={meta}, entering MOVE')
            return

        elif self.state == self.MOVE:
            self._read_target_yaw()
            dx = target[0] - self.current_pos[0]
            dy = target[1] - self.current_pos[1]
            angle_error = self._normalize_angle(self.target_yaw - self.current_yaw)

            arrived = abs(dx) < self.arrive_threshold and \
                      abs(dy) < self.arrive_threshold and \
                      abs(angle_error) < self.angle_threshold
            if arrived:
                meta = self._get_current_meta()
                self.get_logger().info(
                    f'MOVE: roughly arrived! idx={self.current_target_index}/{len(self.current_path.poses)}, '
                    f'meta={meta}, entering ADJ'
                )
                self.state = self.ADJ
                self._reset_pids()
                return

            vel_forward, vel_side, vel_angle = 0, 0, 0
            if abs(angle_error) >= self.angle_threshold and self.suspension_state in [0, 14, 21]:
                vel_angle = self.pid_angle.compute(angle_error, dt)
                if vel_angle > 0: vel_angle = max(0.15, min(self._p('max_angular'), vel_angle))
                elif vel_angle < 0: vel_angle = min(-0.15, max(-self._p('max_angular'), vel_angle))
            else:
                self.angle_ready = True

            if abs(math.degrees(self.target_yaw)) < 1e-6: dis_forward, dis_side = dx, dy
            elif abs(math.degrees(self.target_yaw) - 90) < 1e-6: dis_forward, dis_side = dy, -dx
            elif abs(math.degrees(self.target_yaw) + 90) < 1e-6: dis_forward, dis_side = -dy, dx
            else: dis_forward, dis_side = -dx, -dy

            vel_forward = self.pid_dist.compute(dis_forward, dt)
            if abs(dis_forward) >= self.arrive_threshold:
                if vel_forward > 0: vel_forward = max(0.15, min(self._p('max_vel'), vel_forward))
                elif vel_forward < 0: vel_forward = min(-0.15, max(-self._p('max_vel'), vel_forward))
            else: vel_forward = 0.0

            vel_side = self.pid_dist.compute(dis_side, dt)
            if abs(dis_side) >= self.arrive_threshold:
                if vel_side > 0: vel_side = max(0.15, min(self._p('max_vel'), vel_side))
                elif vel_side < 0: vel_side = min(-0.15, max(-self._p('max_vel'), vel_side))
            else: vel_side = 0.0

            if not self.angle_ready:
                self._publish_cmd(0, 0, vel_angle)
                self.get_logger().info(
                    f'MOVE: pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f},{math.degrees(self.current_yaw):.1f}deg) '
                    f'target=({target[0]:.2f},{target[1]:.2f},{math.degrees(self.target_yaw):.1f}deg) '
                    f'err=({dx:.2f},{dy:.2f},{math.degrees(angle_error):.2f}deg) vel=(0,0,{vel_angle:.3f}) angle_not_ready'
                )
                return

            self._publish_cmd(vel_forward, vel_side, vel_angle)
            self.get_logger().info(
                f'MOVE: pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f},{math.degrees(self.current_yaw):.1f}deg) '
                f'target=({target[0]:.2f},{target[1]:.2f},{math.degrees(self.target_yaw):.1f}deg) '
                f'err=({dx:.2f},{dy:.2f},{math.degrees(angle_error):.2f}deg) vel=({vel_forward:.3f},{vel_side:.3f},{vel_angle:.3f})'
            )

        elif self.state == self.ADJ:
            self._read_target_yaw()
            dx = target[0] - self.current_pos[0]
            dy = target[1] - self.current_pos[1]
            angle_error = self._normalize_angle(self.target_yaw - self.current_yaw)

            dist_thresh, angle_thresh = self._get_adj_thresholds()
            meta = self._get_current_meta()

            arrived = abs(dx) < dist_thresh and abs(dy) < dist_thresh and abs(angle_error) < angle_thresh
            if arrived:
                self.get_logger().info(
                    f'ADJ: arrived! idx={self.current_target_index}/{len(self.current_path.poses)}, meta={meta}, '
                    f'thresh=({dist_thresh:.3f},{angle_thresh:.3f})'
                )
                if meta == 0:
                    self._advance_to_next_target(skip_hold=True)
                else:
                    self._advance_to_next_target(skip_hold=False)
                return

            vel_forward, vel_side, vel_angle = 0, 0, 0
            if abs(angle_error) >= angle_thresh and self.suspension_state in [0, 14, 21]:
                vel_angle = self.pid_adj_angle.compute(angle_error, dt)
                if vel_angle > 0: vel_angle = max(0.15, min(self._p('max_adj_angular'), vel_angle))
                elif vel_angle < 0: vel_angle = min(-0.15, max(-self._p('max_adj_angular'), vel_angle))

            if abs(math.degrees(self.target_yaw)) < 1e-6: dis_forward, dis_side = dx, dy
            elif abs(math.degrees(self.target_yaw) - 90) < 1e-6: dis_forward, dis_side = dy, -dx
            elif abs(math.degrees(self.target_yaw) + 90) < 1e-6: dis_forward, dis_side = -dy, dx
            else: dis_forward, dis_side = -dx, -dy

            vel_forward = self.pid_adj_dist.compute(dis_forward, dt)
            if abs(dis_forward) >= dist_thresh:
                if vel_forward > 0: vel_forward = max(0.15, min(self._p('max_adj_vel'), vel_forward))
                elif vel_forward < 0: vel_forward = min(-0.15, max(-self._p('max_adj_vel'), vel_forward))
            else: vel_forward = 0.0

            vel_side = self.pid_adj_dist.compute(dis_side, dt)
            if abs(dis_side) >= dist_thresh:
                if vel_side > 0: vel_side = max(0.15, min(self._p('max_adj_vel'), vel_side))
                elif vel_side < 0: vel_side = min(-0.15, max(-self._p('max_adj_vel'), vel_side))
            else: vel_side = 0.0

            self._publish_cmd(vel_forward, vel_side, vel_angle)
            self.get_logger().info(
                f'ADJ: meta={meta} pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f},{math.degrees(self.current_yaw):.1f}deg) '
                f'target=({target[0]:.2f},{target[1]:.2f},{math.degrees(self.target_yaw):.1f}deg) '
                f'err=({dx:.2f},{dy:.2f},{math.degrees(angle_error):.2f}deg) vel=({vel_forward:.3f},{vel_side:.3f},{vel_angle:.3f})'
            )

    # =======================================================================
    #  全向模式
    # =======================================================================

    def _control_omni(self, target):
        dt = 1.0 / self.control_frequency

        if self.state == self.HOLD:
            self._stop()
            meta = self._get_current_meta()
            if not self.is_first_hold and meta == 0:
                self._advance_to_next_target(skip_hold=True)
                return

            if self.can_go:
                self.is_first_hold = False
                self.state = self.MOVE
                self._reset_pids()
                self.get_logger().info(f'HOLD: can_go=True, meta={meta}, entering MOVE')
            return

        elif self.state == self.MOVE:
            self._read_target_yaw()
            dx = target[0] - self.current_pos[0]
            dy = target[1] - self.current_pos[1]
            angle_error = self._normalize_angle(-self.current_yaw)

            arrived = abs(dx) < self.arrive_threshold and \
                      abs(dy) < self.arrive_threshold and \
                      abs(angle_error) < self.angle_threshold
            if arrived:
                meta = self._get_current_meta()
                self.get_logger().info(
                    f'MOVE: roughly arrived! idx={self.current_target_index}/{len(self.current_path.poses)}, '
                    f'meta={meta}, entering ADJ'
                )
                self.state = self.ADJ
                self._reset_pids()
                return

            vel_x, vel_y, vel_angle = 0, 0, 0
            if abs(angle_error) >= self.angle_threshold and self.suspension_state in [0, 14, 21]:
                vel_angle = self.pid_angle.compute(angle_error, dt)
                if vel_angle > 0: vel_angle = max(0.15, min(self._p('max_angular'), vel_angle))
                elif vel_angle < 0: vel_angle = min(-0.15, max(-self._p('max_angular'), vel_angle))

            vel_x = self.pid_dist.compute(dx, dt)
            if abs(dx) >= self.arrive_threshold:
                if vel_x > 0: vel_x = max(0.15, min(self._p('max_vel'), vel_x))
                elif vel_x < 0: vel_x = min(-0.15, max(-self._p('max_vel'), vel_x))
            else: vel_x = 0.0

            vel_y = self.pid_dist.compute(dy, dt)
            if abs(dy) >= self.arrive_threshold:
                if vel_y > 0: vel_y = max(0.15, min(self._p('max_vel'), vel_y))
                elif vel_y < 0: vel_y = min(-0.15, max(-self._p('max_vel'), vel_y))
            else: vel_y = 0.0

            self._publish_cmd(vel_x, vel_y, vel_angle)
            self.get_logger().info(
                f'MOVE: pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f},{math.degrees(self.current_yaw):.1f}deg) '
                f'target=({target[0]:.2f},{target[1]:.2f}) '
                f'err=({dx:.2f},{dy:.2f},{math.degrees(angle_error):.2f}deg) vel=({vel_x:.3f},{vel_y:.3f},{vel_angle:.3f})'
            )

        elif self.state == self.ADJ:
            self._read_target_yaw()
            dx = target[0] - self.current_pos[0]
            dy = target[1] - self.current_pos[1]
            angle_error = self._normalize_angle(-self.current_yaw)

            dist_thresh, angle_thresh = self._get_adj_thresholds()
            meta = self._get_current_meta()

            arrived = abs(dx) < dist_thresh and abs(dy) < dist_thresh and abs(angle_error) < angle_thresh
            if arrived:
                self.get_logger().info(
                    f'ADJ: arrived! idx={self.current_target_index}/{len(self.current_path.poses)}, meta={meta}, '
                    f'thresh=({dist_thresh:.3f},{angle_thresh:.3f})'
                )
                if meta == 0:
                    self._advance_to_next_target(skip_hold=True)
                else:
                    self._advance_to_next_target(skip_hold=False)
                return

            vel_x, vel_y, vel_angle = 0, 0, 0
            if abs(angle_error) >= angle_thresh and self.suspension_state in [0, 14, 21]:
                vel_angle = self.pid_adj_angle.compute(angle_error, dt)
                if vel_angle > 0: vel_angle = max(0.15, min(self._p('max_adj_angular'), vel_angle))
                elif vel_angle < 0: vel_angle = min(-0.15, max(-self._p('max_adj_angular'), vel_angle))

            vel_x = self.pid_adj_dist.compute(dx, dt)
            if abs(dx) >= dist_thresh:
                if vel_x > 0: vel_x = max(0.15, min(self._p('max_adj_vel'), vel_x))
                elif vel_x < 0: vel_x = min(-0.15, max(-self._p('max_adj_vel'), vel_x))
            else: vel_x = 0.0

            vel_y = self.pid_adj_dist.compute(dy, dt)
            if abs(dy) >= dist_thresh:
                if vel_y > 0: vel_y = max(0.15, min(self._p('max_adj_vel'), vel_y))
                elif vel_y < 0: vel_y = min(-0.15, max(-self._p('max_adj_vel'), vel_y))
            else: vel_y = 0.0

            self._publish_cmd(vel_x, vel_y, vel_angle)
            self.get_logger().info(
                f'ADJ: meta={meta} pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f},{math.degrees(self.current_yaw):.1f}deg) '
                f'target=({target[0]:.2f},{target[1]:.2f}) '
                f'err=({dx:.2f},{dy:.2f},{math.degrees(angle_error):.2f}deg) vel=({vel_x:.3f},{vel_y:.3f},{vel_angle:.3f})'
            )

    def _stop(self):
        self._publish_cmd(0.0, 0.0, 0.0)

    def _publish_cmd(self, vx, vy, vyaw):
        msg = Float32MultiArray()
        msg.data = [float(vx), float(vy), float(vyaw)]
        self.cmd_pub.publish(msg)

    def _publish_can_do(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
