#!/usr/bin/env python3
"""
G60Pro Nav2 导航启动文件
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用模拟时间'
    )

    # 动态获取包路径，避免硬编码
    robot_navigation_share = get_package_share_directory('robot_navigation')
    robot_bringup_share = get_package_share_directory('robot_bringup')

    map_file_default = os.path.join(robot_bringup_share, 'maps', 'g60pro_map.yaml')
    params_file = os.path.join(robot_navigation_share, 'config', 'nav2_params.yaml')

    map_file = DeclareLaunchArgument(
        'map',
        default_value=map_file_default,
        description='地图文件路径'
    )

    # Nav2 启动文件
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': params_file,
            'map': LaunchConfiguration('map')
        }.items()
    )

    return LaunchDescription([
        use_sim_time,
        map_file,
        nav2_launch
    ])
