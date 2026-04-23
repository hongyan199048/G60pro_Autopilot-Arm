#!/usr/bin/env python3
"""
G60Pro Nav2 导航启动文件 — 仿真（Gazebo）
pointcloud_to_laserscan 高度参数与 cartographer_sim.lua 对齐
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
        default_value='true',
        description='使用模拟时间'
    )

    robot_navigation_share = get_package_share_directory('robot_navigation')

    # 使用顶层 maps 目录（工作空间根目录）
    workspace_root = os.path.abspath(os.path.join(robot_navigation_share, '../../../../'))
    map_file_default = os.path.join(workspace_root, 'maps', 'g60pro_sim_map_v5.yaml')
    params_file = os.path.join(robot_navigation_share, 'config', 'nav2_params_sim.yaml')

    map_file = DeclareLaunchArgument(
        'map',
        default_value=map_file_default,
        description='地图文件路径'
    )

    # 3D 点云 → 2D 激光扫描，供 AMCL 和代价地图使用
    # 高度相对 base_footprint（地面 Z=0），与 cartographer_sim.lua min_z/max_z 对齐
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.5,
            'min_height': 0.1,    # 与 cartographer_sim.lua min_z=0.1 对齐
            'max_height': 2.5,    # 与 cartographer_sim.lua max_z=2.5 对齐
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00436,
            'scan_time': 0.1,
            'range_min': 0.5,     # 与 cartographer_sim.lua min_range=0.5 对齐
            'range_max': 30.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/lidar/multi/points'),
            ('scan', '/scan'),
        ],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/opt/ros/humble/share/nav2_bringup/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': params_file,
            'map': LaunchConfiguration('map'),
            'slam': 'False',
            'autostart': 'True'
        }.items()
    )

    return LaunchDescription([
        use_sim_time,
        map_file,
        pointcloud_to_laserscan,
        nav2_launch,
    ])
