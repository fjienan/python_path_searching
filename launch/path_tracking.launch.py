#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    启动路径跟踪系统的launch文件

    同时启动：
    - astar_planner: A*路径规划节点
    - omnidirectional_tracker: 全向轮跟踪节点
    - coordinate_transform: 坐标转换节点
    - motion_control: 运动控制节点
    - odom_simulator: 里程计模拟器
    """

    # A*路径规划节点
    astar_planner_node = Node(
        package='python_path_searching',
        executable='astar_planner_node.py',
        name='astar_planner',
        output='screen',
        parameters=[]
    )

    # 全向轮跟踪节点
    omnidirectional_tracker_node = Node(
        package='python_path_searching',
        executable='omnidirectional_tracker_node.py',
        name='omnidirectional_tracker',
        output='screen',
        parameters=[
            {'kp_linear': 1.0},
            {'ki_linear': 0.0},
            {'kd_linear': 0.1},
            {'kp_angular': 1.0},
            {'ki_angular': 0.0},
            {'kd_angular': 0.1},
            {'max_linear_vel': 0.5},
            {'max_angular_vel': 1.0},
            {'control_frequency': 50.0},
            {'target_distance_threshold': 0.1},
            {'kfs_check_distance': 0.7}
        ]
    )

    # 坐标转换节点
    coordinate_transform_node = Node(
        package='python_path_searching',
        executable='coordinate_transform_node.py',
        name='coordinate_transform',
        output='screen',
        parameters=[
            {'robot_offset_x': 0.0},
            {'robot_offset_y': 0.0},
            {'initial_yaw': 0.0}
        ]
    )

    # 运动控制节点
    motion_control_node = Node(
        package='python_path_searching',
        executable='motion_control_node.py',
        name='motion_control',
        output='screen',
        parameters=[
            {'max_linear_vel': 0.5},
            {'max_angular_vel': 1.0},
            {'control_frequency': 50.0},
            {'target_distance_threshold': 0.1},
            {'map2_origin_x': 3.2},
            {'map2_origin_y': 1.2},
            {'grid_resolution': 1.2}
        ]
    )

    # 里程计模拟器节点
    odom_simulator_node = Node(
        package='python_path_searching',
        executable='odom_simulator.py',
        name='odom_simulator',
        output='screen',
        parameters
       =[
            {'odom_frame': 'map'},
            {'base_frame': 'base_link'},
            {'publish_rate': 50.0}
        ]
    )

    return LaunchDescription([
        astar_planner_node,
        omnidirectional_tracker_node,
        coordinate_transform_node,
        motion_control_node,
        odom_simulator_node,
    ])
