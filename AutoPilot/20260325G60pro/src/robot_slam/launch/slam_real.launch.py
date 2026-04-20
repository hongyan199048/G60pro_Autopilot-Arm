#!/usr/bin/env python3
"""
G60Pro Cartographer SLAM 启动文件 — 实车（Helios16）
订阅 /lidar/rs16/points，加载 cartographer_real.lua
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
        cartographer,
        cartographer_occupancy_grid
    ])
