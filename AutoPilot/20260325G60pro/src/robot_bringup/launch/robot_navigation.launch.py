#!/usr/bin/env python3
"""
G60Pro 导航启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用模拟时间'
    )

    # 1. 机器人描述
    description_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('robot_description'), 'launch', 'description.launch.py'),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 2. 底盘控制
    base_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('robot_base'), 'launch', 'base.launch.py'),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 3. 传感器
    sensors_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('robot_sensors'), 'launch', 'sensors.launch.py'),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 4. 定位
    localization_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('robot_localization'), 'launch', 'ekf.launch.py'),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 5. Nav2 导航
    nav_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('robot_navigation'), 'launch', 'navigation.launch.py'),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 6. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('robot_bringup'), 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        description_launch,
        base_launch,
        sensors_launch,
        localization_launch,
        nav_launch,
        # rviz_node
    ])