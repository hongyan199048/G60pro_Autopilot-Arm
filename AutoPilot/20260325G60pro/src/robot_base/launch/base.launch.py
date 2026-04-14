#!/usr/bin/env python3
"""
G60Pro 底盘控制启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='模拟模式'
    )

    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='1.0',
        description='最大线速度 (m/s)'
    )

    max_angular_arg = DeclareLaunchArgument(
        'max_angular',
        default_value='1.0',
        description='最大角速度 (rad/s)'
    )

    # 底盘节点
    base_node = Node(
        package='robot_base',
        executable='robot_base_node',
        name='robot_base',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wheel_radius': 0.15,
            'wheelbase_x': 0.75,
            'wheelbase_y': 0.5,
            'max_velocity': LaunchConfiguration('max_velocity'),
            'max_angular': LaunchConfiguration('max_angular'),
            'use_sim': LaunchConfiguration('use_sim')
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_sim_arg,
        max_velocity_arg,
        max_angular_arg,
        base_node
    ])