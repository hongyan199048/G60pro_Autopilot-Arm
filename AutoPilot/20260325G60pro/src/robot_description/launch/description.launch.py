#!/usr/bin/env python3
"""
G60Pro 机器人描述启动文件
发布 URDF 模型和 TF 变换
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from xacro import process_file


def generate_launch_description():
    # 获取包路径
    pkg_name = 'robot_description'
    pkg_dir = get_package_share_directory(pkg_name)

    # URDF xacro 文件路径
    xacro_file = os.path.join(pkg_dir, 'urdf', 'g60pro.urdf.xacro')

    # 声明参数
    lidar_type_arg = DeclareLaunchArgument(
        'lidar_type',
        default_value='robosense_16',
        description='激光雷达类型: robosense_16 或 livox_mid360'
    )

    # 简化处理：直接使用默认参数处理 xacro
    doc = process_file(xacro_file, mappings={'lidar_type': 'robosense_16'})
    robot_description_content = doc.toxml()

    # 机器人状态发布节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        lidar_type_arg,
        robot_state_publisher_node
    ])