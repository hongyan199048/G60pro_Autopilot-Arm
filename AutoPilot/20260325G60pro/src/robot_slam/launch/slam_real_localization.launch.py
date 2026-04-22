#!/usr/bin/env python3
"""
G60Pro Cartographer 纯定位模式启动文件 — 实车
加载 .pbstream 地图文件，在已保存地图内做 scan matching 定位
不再构建新地图

用法：
  ros2 launch robot_slam slam_real_localization.launch.py \
    pbstream_file:=/path/to/g60pro_v6.pbstream

TF 链：
  map → base_footprint（由 Cartographer 发布）
  base_footprint → base_link（由 robot_state_publisher 从 URDF 发布）
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用模拟时间（实车固定为 false）'
    )

    pbstream_file = DeclareLaunchArgument(
        'pbstream_file',
        description='.pbstream 地图文件路径（必填）'
    )

    slam_config_dir = FindPackageShare('robot_slam').find('robot_slam')

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=[
            '-configuration_directory', slam_config_dir + '/config',
            '-configuration_basename', 'cartographer_real_localization.lua',
            '-load_state_filename', LaunchConfiguration('pbstream_file'),
        ],
        remappings=[
            ('points2', '/lidar/rs16/points'),
            ('imu', 'imu'),
            ('odom', 'odom')
        ]
    )

    return LaunchDescription([
        use_sim_time,
        pbstream_file,
        cartographer
    ])
