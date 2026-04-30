#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'python_path_searching'
    pkg_share = get_package_share_directory(pkg_name)
    config_file = os.path.join(pkg_share, 'config', 'para.yaml')

    return LaunchDescription([
        Node(package=pkg_name, executable='astar_planner_node', name='astar_planner',
             output='screen', parameters=[config_file]),
        Node(package=pkg_name, executable='tracker_node', name='tracker',
             output='screen', parameters=[config_file]),

        # === 仿真模式：取消注释以下节点 ===
        # 仿真时需要修改 para.yaml 中的 topics.odom_world 为本节点发布的 topic，
        # 默认 tracker 订阅 /odom_world，与本节点发布 topic 一致，无需额外修改。
        # Node(package=pkg_name, executable='odom_simulator', name='odom_simulator',
        #      output='screen', parameters=[config_file]),

        # === 真实 SLAM 模式：注释掉上面的 odom_simulator 节点 ===
        # 确保外部 SLAM 系统的 odometry 发布的 topic 与 tracker 订阅的
        # topics.odom_world (即 /odom_world) 一致，必要时修改 para.yaml。
    ])
