#!/usr/bin/env python3
"""
path_tracking.launch.py

用法：注释 / 取消注释各 Node 块来控制节点启停。
默认全部启用（仿真模式）。

真实 SLAM 模式：注释掉 odom_simulator，
并确保外部 SLAM 的 odometry topic 与 tracker 订阅的 topic 一致。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'python_path_searching'
    pkg_share = get_package_share_directory(pkg_name)
    params_file = f'{pkg_share}/config/para.yaml'

    return LaunchDescription([
        # === A* 路径规划 ===
        Node(
            package=pkg_name,
            executable='astar_planner_node',
            name='astar_planner',
            output='screen',
            parameters=[params_file],
        ),

        # === 路径决策（can_go 控制）===
     #    Node(
     #        package=pkg_name,
     #        executable='path_decision_node',
     #        name='path_decision',
     #        output='screen',
     #        parameters=[params_file],
     #    ),

        # === PID 轨迹跟踪 ===
        Node(
            package=pkg_name,
            executable='tracker_node',
            name='tracker',
            output='screen',
            parameters=[params_file],
        ),

        # === 里程计仿真（仅仿真模式启用）===
        # 真实 SLAM 时注释掉本节点
        Node(
            package=pkg_name,
            executable='odom_simulator',
            name='odom_simulator',
            output='screen',
            parameters=[params_file],
        ),
    ])
