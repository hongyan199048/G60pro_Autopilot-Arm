#!/usr/bin/env python3
"""
G60Pro Cartographer 纯定位启动文件 — 实车（Helios16）
仅定位：订阅 /lidar/rs16/points，发布 map→base_footprint TF，
不发布实时 /map（避免与 map_server 静态地图冲突）。

TF 链：map → base_footprint → base_link → ...
"""

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

    slam_config_dir = FindPackageShare('robot_slam').find('robot_slam')

    # TF 链说明：
    # Cartographer 发布：map → base_footprint
    # robot_state_publisher 从 URDF 发布：base_footprint → base_link → rs16_link → ...
    # 不需要任何额外静态 TF，无冲突

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
            '-configuration_basename', 'cartographer_real.lua'
        ],
        remappings=[
            ('points2', '/lidar/rs16/points'),
            ('imu', 'imu'),
            ('odom', 'odom')
        ]
    )

    return LaunchDescription([
        use_sim_time,
        cartographer
    ])
