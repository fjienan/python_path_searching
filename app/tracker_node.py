#!/usr/bin/env python3
"""
轨迹跟踪节点（逐段 PID 控制）

状态机流程：
  单向模式：TURN → HOLD → MOVE → TURN → ...
           - TURN: 转到目标角度（can_go 不影响）
           - HOLD: 停在关键点，等 can_go=True 才继续
           - MOVE: 前进到目标点，偏离角度则切回 TURN

  全向模式：TURN → HOLD → MOVE → TURN → ...
           - TURN: 转到目标角度，转向与平移完全分开
           - HOLD: 停在关键点，等 can_go=True 才继续
           - MOVE: 直线移动到目标点，偏离角度则切回 TURN
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, Float32
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import math
import numpy as np

from core.pid_controller import PIDController
from core.transform_utils import yaw_from_quaternion


class TrackerNode(Node):

    TURN, MOVE, HOLD = 0, 1, 2

    def __init__(self):
        super().__init__('tracker')

        self.declare_parameter('motion_type', 'unidirectional')
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('arrive_threshold', 0.12)

        self.declare_parameter('topics.planning_path', '/planning/path')
        self.declare_parameter('topics.odom_world', '/odom_world')
        self.declare_parameter('topics.can_go', '/can_go')
        self.declare_parameter('topics.cmd_vel', '/cmd_vel')
        self.declare_parameter('topics.tracker_direction', '/direction')
        self.declare_parameter('topics.target_yaw_deg', '/target_yaw_deg')

        self.declare_parameter('kp_x', 3.0)
        self.declare_parameter('ki_x', 0.0)
        self.declare_parameter('kd_x', 0.2)
        self.declare_parameter('kp_y', 3.0)
        self.declare_parameter('ki_y', 0.0)
        self.declare_parameter('kd_y', 0.2)
        self.declare_parameter('kp_yaw', 5.0)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.3)

        self.declare_parameter('kp_dist', 2.5)
        self.declare_parameter('ki_dist', 0.0)
        self.declare_parameter('kd_dist', 0.1)
        self.declare_parameter('kp_angle', 4.0)
        self.declare_parameter('ki_angle', 0.0)
        self.declare_parameter('kd_angle', 0.2)

        self.declare_parameter('max_vel_x', 0.5)
        self.declare_parameter('max_vel_y', 0.5)
        self.declare_parameter('max_vel_yaw', 2.0)
        self.declare_parameter('max_vel_forward', 0.5)
        self.declare_parameter('max_angular', 2.0)

        self.motion_type = self.get_parameter('motion_type').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.arrive_threshold = self.get_parameter('arrive_threshold').value

        self.get_logger().info(f'Tracker Node started, motion_type={self.motion_type}')

        self.current_path = None
        self.current_target_index = 0
        self._move_first_iteration = False
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.have_path = False
        self.have_odom = False
        self.can_go = False
        self.prev_can_go = False

        self.state = self.TURN
        self.state_switched = False

        self._init_pids()
        self._init_subscribers()
        self._init_publishers()

        self.control_timer = self.create_timer(1.0 / self.control_frequency, self._control_callback)

    def _p(self, name, default=None):
        val = self.get_parameter(name).value
        return val if val is not None else default

    def _init_pids(self):
        if self.motion_type == 'omnidirectional':
            self.pid_x = PIDController(
                self._p('kp_x'), self._p('ki_x'), self._p('kd_x'),
                self._p('max_vel_x')
            )
            self.pid_y = PIDController(
                self._p('kp_y'), self._p('ki_y'), self._p('kd_y'),
                self._p('max_vel_y')
            )
            self.pid_yaw = PIDController(
                self._p('kp_yaw'), self._p('ki_yaw'), self._p('kd_yaw'),
                self._p('max_vel_yaw')
            )
        else:
            self.pid_angle = PIDController(
                self._p('kp_angle'), self._p('ki_angle'), self._p('kd_angle'),
                self._p('max_angular')
            )
            self.pid_dist = PIDController(
                self._p('kp_dist'), self._p('ki_dist'), self._p('kd_dist'),
                self._p('max_vel_forward')
            )

    def _init_subscribers(self):
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        path_topic = self._p('topics.planning_path', '/planning/path')
        odom_topic = self._p('topics.odom_world', '/odom_world')
        can_go_topic = self._p('topics.can_go', '/can_go')
        self.path_sub = self.create_subscription(Path, path_topic, self._path_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.can_go_sub = self.create_subscription(Bool, can_go_topic, self._can_go_callback, 10)

    def _init_publishers(self):
        cmd_topic = self._p('topics.cmd_vel', '/cmd_vel')
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        dir_topic = self._p('topics.tracker_direction', '/direction')
        yaw_topic = self._p('topics.target_yaw_deg', '/target_yaw_deg')
        self.dir_pub = self.create_publisher(Int32, dir_topic, 10)
        self.yaw_pub = self.create_publisher(Float32, yaw_topic, 10)

    def _path_callback(self, msg):
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return

        new_target_idx = 1
        if new_target_idx >= len(msg.poses):
            self.get_logger().warn('Path too short after skipping first point')
            return

        # 如果已有路径且当前目标点相同，不重置（避免 planner 重复发同一路径时打断跟踪）
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
        self.current_target_index = new_target_idx
        self.have_path = True
        self._move_first_iteration = True
        self.state = self.TURN
        self._reset_pids()
        self._read_target_yaw()
        self._publish_direction()
        self.yaw_pub.publish(Float32(data=math.degrees(self.target_yaw)))
        self.get_logger().info(f'Received path with {len(msg.poses)} points')

    def _odom_callback(self, msg):
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True

    def _can_go_callback(self, msg):
        self.prev_can_go = self.can_go
        self.can_go = msg.data

    def _was_can_go_just_disabled(self):
        """检测 can_go 是否刚刚从 True 变为 False（即本帧从允许变为禁止）"""
        return self.prev_can_go and not self.can_go

    def _read_target_yaw(self):
        if self.current_path is None or self.current_target_index >= len(self.current_path.poses):
            return

        if self.motion_type == 'unidirectional':
            # 单向模式：每个路点的 yaw 是来时方向
            # 到达前 look ahead：距离下一个点 < 1.0m 时提前读取下个点的 yaw，转向下一个点的来时方向
            idx = self.current_target_index
            if idx + 1 < len(self.current_path.poses):
                next_pose = self.current_path.poses[idx + 1].pose
                dx = next_pose.position.x - self.current_pos[0]
                dy = next_pose.position.y - self.current_pos[1]
                if math.hypot(dx, dy) < 1.0:
                    self.target_yaw = yaw_from_quaternion(next_pose.orientation)
                    return
            # 否则读取当前点的 yaw（来时方向）
            self.target_yaw = yaw_from_quaternion(
                self.current_path.poses[idx].pose.orientation
            )
        else:
            # 全向模式：直接读取路径自带的姿态
            self.target_yaw = yaw_from_quaternion(
                self.current_path.poses[self.current_target_index].pose.orientation
            )

    def _reset_pids(self):
        if self.motion_type == 'omnidirectional':
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_yaw.reset()
        else:
            self.pid_angle.reset()
            self.pid_dist.reset()

    def _publish_direction(self):
        """在设置新目标yaw时立即发布转向方向（左转-1/直走0/右转1）"""
        diff = self._normalize_angle(self.target_yaw - self.current_yaw)
        diff_deg = math.degrees(diff)
        if diff_deg > 2.0:
            direction = -1
        elif diff_deg < -2.0:
            direction = 1
        else:
            direction = 0
        self.dir_pub.publish(Int32(data=direction))
        self.get_logger().info(f'PUBLISH direction={direction} target_yaw_deg={math.degrees(self.target_yaw):.2f} diff={diff_deg:.2f}')

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
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

    def _move_to_next_target(self):
        self.current_target_index += 1
        self.can_go = False
        self.prev_can_go = False
        self.get_logger().info(f'_move_to_next_target: new_idx={self.current_target_index} total={len(self.current_path.poses)}, waiting for can_go')
        if self.current_target_index >= len(self.current_path.poses):
            self.get_logger().info('All targets reached, stopping')
            return False
        # 直接读下一个点的 yaw（来时方向），不再依赖几何计算
        self.target_yaw = yaw_from_quaternion(
            self.current_path.poses[self.current_target_index].pose.orientation
        )
        self._reset_pids()
        self.state = self.TURN
        self._publish_direction()  # 立即发布转向方向
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
            else:
                self._publish_cmd(0.0, 0.0, angular_vel)
                return

        elif self.state == self.HOLD:
            self._stop()
            if self.can_go:
                self.state = self.MOVE
                self._move_first_iteration = True
                self._reset_pids()
                self.get_logger().info('can_go=True, entering MOVE')

        elif self.state == self.MOVE:
            skip_arrival_check = self._move_first_iteration
            self._move_first_iteration = False

            arrived = self._check_arrived(target)
            self.get_logger().info(f'MOVE check: skip={skip_arrival_check} arrived={arrived} can_go={self.can_go}')

            if not skip_arrival_check and arrived:
                self.get_logger().info(f'MOVE: arrived! idx={self.current_target_index}/{len(self.current_path.poses)}, calling _move_to_next_target')
                if not self._move_to_next_target():
                    self._stop()
                    self.have_path = False
                    self.get_logger().info('MOVE: all done, stopped')
                return

            self._read_target_yaw()
            angle_error = self._normalize_angle(self.target_yaw - self.current_yaw)
            new_angle_error = abs(angle_error)

            dx = target[0] - self.current_pos[0]
            dy = target[1] - self.current_pos[1]
            dist = math.sqrt(dx * dx + dy * dy)

            self.get_logger().info(f'MOVE: pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f}) target=({target[0]:.2f},{target[1]:.2f}) dist={dist:.3f} angle_err={new_angle_error:.3f}')

            forward_vel = self.pid_dist.compute(dist, dt)

            # 核心魔法：如果距离还没达到容忍阈值，绝不允许速度低于电机的启动死区
            if dist >= self.arrive_threshold:
                if forward_vel > 0:
                    forward_vel = max(0.15, min(self._p('max_vel_forward'), forward_vel))
            else:
                forward_vel = 0.0

            angular_vel = self.pid_angle.compute(angle_error, dt)
            angular_vel = max(-self._p('max_angular'), min(self._p('max_angular'), angular_vel))

            self._publish_cmd(forward_vel, 0.0, angular_vel)

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
                self.get_logger().info('Angle aligned, entering HOLD')
                return
            else:
                self._publish_cmd(0.0, 0.0, vyaw)
                return

        elif self.state == self.HOLD:
            self._stop()
            if self.can_go:
                self.state = self.MOVE
                self._move_first_iteration = True
                self._reset_pids()
                self.get_logger().info('can_go=True, entering MOVE')

        elif self.state == self.MOVE:
            skip_arrival_check = self._move_first_iteration
            self._move_first_iteration = False

            self._read_target_yaw()
            angle_error = self._normalize_angle(self.target_yaw - self.current_yaw)
            angle_err = abs(angle_error)

            dx = target[0] - self.current_pos[0]
            dy = target[1] - self.current_pos[1]
            dist = math.sqrt(dx * dx + dy * dy)

            if not skip_arrival_check and dist < self.arrive_threshold:
                if not self._move_to_next_target():
                    self._stop()
                    self.have_path = False
                    self.get_logger().info('MOVE: all done, stopped')
                return

            self.get_logger().info(f'MOVE: pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f}) target=({target[0]:.2f},{target[1]:.2f}) dist={dist:.3f} angle_err={angle_err:.3f}')

            progress = min(1.0, max(0.0, (dist - (self.arrive_threshold - 0.05)) / 0.05))
            vx = dx / dist * self._p('max_vel_x') * progress
            vy = dy / dist * self._p('max_vel_y') * progress

            vx = max(-self._p('max_vel_x'), min(self._p('max_vel_x'), vx))
            vy = max(-self._p('max_vel_y'), min(self._p('max_vel_y'), vy))

            self._publish_cmd(vx, vy, 0.0)

    def _stop(self):
        self._publish_cmd(0.0, 0.0, 0.0)

    def _publish_cmd(self, vx, vy, vyaw):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(vyaw)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
