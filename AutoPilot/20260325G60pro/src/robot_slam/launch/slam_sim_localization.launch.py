#!/usr/bin/env python3
"""
G60Pro Cartographer 纯定位模式启动文件 — 仿真（Gazebo）
加载 .pbstream 地图，在已保存地图上做 scan matching 定位，不复写子图

用法：
  ros2 launch robot_slam slam_sim_localization.launch.py \
    pbstream_file:=/path/to/maps/g60pro_v10.pbstream

TF 链：map → base_footprint（Cartographer 独家发布）
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用模拟时间（仿真固定为 true）'
    )

    pbstream_file = DeclareLaunchArgument(
        'pbstream_file',
        description='.pbstream 地图文件路径（必填）'
    )

    pkg_share = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    slam_config_dir = os.path.join(pkg_share, 'config')

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=[
            '-configuration_directory', slam_config_dir,
            '-configuration_basename', 'cartographer_sim_localization.lua',
            '-load_state_filename', LaunchConfiguration('pbstream_file'),
        ],
        remappings=[
            ('points2', '/lidar/multi/points'),
            ('imu', 'imu'),
            ('odom', 'odom')
        ]
    )

    cartographer_occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'resolution': 0.05,
            'publish_period_sec': 1.0
        }]
    )

    return LaunchDescription([
        use_sim_time,
        pbstream_file,
        cartographer,
        cartographer_occupancy_grid,
    ])
