#!/usr/bin/env python3
"""
G60Pro CAN 通信启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数
    can1_channel_arg = DeclareLaunchArgument(
        'can1_channel',
        default_value='can0',
        description='CAN1 通道'
    )

    can4_channel_arg = DeclareLaunchArgument(
        'can4_channel',
        default_value='can1',
        description='CAN4 通道'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='模拟模式'
    )

    # CAN 节点
    can_node = Node(
        package='robot_can',
        executable='can_node',
        name='robot_can',
        output='screen',
        parameters=[{
            'can1_channel': LaunchConfiguration('can1_channel'),
            'can4_channel': LaunchConfiguration('can4_channel'),
            'can1_baudrate': 500000,
            'can4_baudrate': 1000000,
            'use_sim': LaunchConfiguration('use_sim')
        }]
    )

    return LaunchDescription([
        can1_channel_arg,
        can4_channel_arg,
        use_sim_arg,
        can_node
    ])