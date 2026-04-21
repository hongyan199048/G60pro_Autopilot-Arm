#!/usr/bin/env python3
"""
G60Pro Nav2 导航启动文件 — 实车（Helios16）
pointcloud_to_laserscan 高度参数与 cartographer_real.lua 对齐

定位由 Cartographer 提供（map→base_footprint），不启动 AMCL。
使用 nav2_no_amcl.launch.py 替代标准 bringup_launch.py。
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
        description='使用模拟时间（实车固定为 false）'
    )

    robot_navigation_share = get_package_share_directory('robot_navigation')
    nav2_launch_file = os.path.join(robot_navigation_share, 'launch', 'nav2_no_amcl.launch.py')

    map_file = DeclareLaunchArgument(
        'map',
        default_value='',
        description='地图文件路径（必须指定，例如 map:=/path/to/map.yaml）'
    )

    params_file = os.path.join(robot_navigation_share, 'config', 'nav2_params_real.yaml')

    # 3D 点云 → 2D 激光扫描，供代价地图使用
    # 高度相对 base_footprint（地面 Z=0），与 cartographer_real.lua 对齐：
    # cartographer_real.lua min_z=-1.44 max_z=0.46（雷达坐标系，Helios16 离地 1.54m）
    # 换算到 base_footprint：min_height=0.1m，max_height=2.0m
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.5,
            'min_height': 0.1,
            'max_height': 2.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00436,
            'scan_time': 0.1,
            'range_min': 1.6,
            'range_max': 30.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/lidar/rs16/points'),
            ('scan', '/scan'),
        ],
    )

    # Nav2 导航节点（无 AMCL 版）
    # map_server 加载静态 .yaml 地图（供全局代价地图）
    # 所有定位 TF 由 Cartographer 提供（map→base_footprint）
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': params_file,
            'map': LaunchConfiguration('map'),
            'autostart': 'True'
        }.items()
    )

    return LaunchDescription([
        use_sim_time,
        map_file,
        pointcloud_to_laserscan,
        nav2_launch,
    ])
