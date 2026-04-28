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
        Node(package=pkg_name, executable='odom_simulator', name='odom_simulator',
             output='screen', parameters=[config_file]),
    ])
