#!/usr/bin/env python3
"""
G60Pro Cartographer SLAM 启动文件 — 仿真（Gazebo）
订阅 /lidar/multi/points，加载 cartographer_sim.lua
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用模拟时间'
    )

    slam_config_dir = get_package_share_directory('robot_slam')

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
            '-configuration_basename', 'cartographer_sim.lua'
        ],
        remappings=[
            ('points2', 'lidar/multi/points'),
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
