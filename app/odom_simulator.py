#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomSimulator(Node):
    """Simple omni-directional base simulator that integrates received cmd_vel."""

    def __init__(self) -> None:
        super().__init__('odom_simulator')

        self.declare_parameter('odom_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('motion_type', 'omnidirectional')
        self.declare_parameter('grid.map_origin', [3.2, 1.2, 0.0])
        self.declare_parameter('grid.grid_resolution', 1.2)
        self.declare_parameter('grid.start_row', -1)
        self.declare_parameter('grid.start_col', 0)

        odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        publish_rate = max(1.0, self.get_parameter('publish_rate').value)
        motion_type = self.get_parameter('motion_type').get_parameter_value().string_value

        self._odom_frame = odom_frame or 'odom'
        self._base_frame = base_frame or 'base_link'
        self._motion_type = motion_type or 'omnidirectional'
        self._dt = 1.0 / publish_rate

        map_origin_raw = self.get_parameter('grid.map_origin').value
        grid_resolution = self.get_parameter('grid.grid_resolution').value
        start_row = self.get_parameter('grid.start_row').value
        start_col = self.get_parameter('grid.start_col').value

        start_x = map_origin_raw[0] + start_row * grid_resolution + 0.5 * grid_resolution
        start_y = map_origin_raw[1] + start_col * grid_resolution + 0.5 * grid_resolution

        self._pose_x = start_x
        self._pose_y = start_y
        self._pose_z = 0.0
        self._yaw = 0.0

        self._vel_cmd: Twist = Twist()

        self._last_update: Optional[Time] = None

        self._cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_callback, 10
        )
        self._initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._initial_pose_callback,
            10,
        )
        self._odom_pub = self.create_publisher(
            Odometry, '/odom_world', 10
        )
        
        # TF broadcaster for odom->base_link transform
        self._tf_broadcaster = TransformBroadcaster(self)
        
        self.create_timer(self._dt, self._publish_odometry)

        self.get_logger().info(
            f'Odom simulator started. Publishing {publish_rate:.1f} Hz on /odom_world, motion_type={self._motion_type}'
        )
        self.get_logger().info(
            f'Publishing TF transform: {self._odom_frame} -> {self._base_frame}'
        )
        self.get_logger().info(
            f'Initial position: [{self._pose_x:.2f}, {self._pose_y:.2f}] (grid [-1][0])'
        )

    def _cmd_callback(self, msg: Twist) -> None:
        self._vel_cmd = msg

    def _initial_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self._pose_x = pose.position.x
        self._pose_y = pose.position.y
        self._pose_z = pose.position.z

        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)

        if norm > 0.0:
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm

            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            self._yaw = math.atan2(siny_cosp, cosy_cosp)
        else:
            self._yaw = 0.0

        self._last_update = msg.header.stamp
        self.get_logger().info(
            f'Received /initialpose: ({self._pose_x:.2f}, {self._pose_y:.2f}, {self._pose_z:.2f}), yaw={math.degrees(self._yaw):.1f} deg'
        )

    def _publish_odometry(self) -> None:
        now = self.get_clock().now().to_msg()
        if self._last_update is None:
            self._last_update = now
            return

        dt = self._dt

        if self._motion_type == 'omnidirectional':
            vx_global = self._vel_cmd.linear.x
            vy_global = self._vel_cmd.linear.y
        else:
            forward_vel = self._vel_cmd.linear.x
            yaw = self._yaw
            vx_global = forward_vel * math.cos(yaw)
            vy_global = forward_vel * math.sin(yaw)

        wz = self._vel_cmd.angular.z

        self._pose_x += vx_global * dt
        self._pose_y += vy_global * dt
        # 平面机器人：固定Z坐标为0
        self._pose_z = 0.0
        self._yaw += wz * dt

        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self._odom_frame
        odom_msg.child_frame_id = self._base_frame

        odom_msg.pose.pose.position.x = self._pose_x
        odom_msg.pose.pose.position.y = self._pose_y
        odom_msg.pose.pose.position.z = self._pose_z

        half_yaw = self._yaw * 0.5
        odom_msg.pose.pose.orientation.z = math.sin(half_yaw)
        odom_msg.pose.pose.orientation.w = math.cos(half_yaw)

        odom_msg.twist.twist = self._vel_cmd

        self._odom_pub.publish(odom_msg)
        
        # Publish TF transform (equivalent to odom message)
        self._publish_tf(now)
    
    def _publish_tf(self, stamp) -> None:
        """发布TF变换，从odom_frame到base_frame"""
        t = TransformStamped()
        
        # 设置时间戳
        t.header.stamp = stamp
        t.header.frame_id = self._odom_frame  # 父坐标系（通常是 'map'）
        t.child_frame_id = self._base_frame   # 子坐标系（通常是 'base_link'）
        
        # 设置平移
        t.transform.translation.x = self._pose_x
        t.transform.translation.y = self._pose_y
        t.transform.translation.z = self._pose_z
        
        # 设置旋转（从yaw角转换为四元数）
        half_yaw = self._yaw * 0.5
        t.transform.rotation.z = math.sin(half_yaw)
        t.transform.rotation.w = math.cos(half_yaw)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        
        # 发布TF
        self._tf_broadcaster.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = OdomSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

