#!/usr/bin/env python3
"""
G60Pro Gazebo 仿真启动文件
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    # 使用软渲染避免 GPU 问题
    os.environ['OGRE_RENDERER'] = 'GL'
    # 禁止 Gazebo 从网络拉取模型数据库（否则启动会卡住导致插件加载失败）
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''
    # 设置 GAZEBO_MODEL_PATH：
    # 1. robot_description mesh（package:// URI 解析）
    # 2. Gazebo 系统模型（ground_plane、sun 等内置模型必须包含）
    desc_share = get_package_share_directory('robot_description')
    extra_model_path = os.path.dirname(desc_share)
    gazebo_system_models = '/usr/share/gazebo-11/models'
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    paths = [p for p in [extra_model_path, gazebo_system_models, existing] if p]
    os.environ['GAZEBO_MODEL_PATH'] = ':'.join(paths)

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间'
    )

    # 1. 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py'),
        launch_arguments={'world': '/opt/ros/humble/share/gazebo_ros/worlds/empty.world'}.items()
    )

    # 2. 启动 Gazebo 客户端 (GUI 模式)
    gazebo_client = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
    )

    # 3. 机器人描述 (URDF) - 包含 robot_state_publisher
    description_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('robot_description'), 'launch', 'description.launch.py'),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 4. Spawn 机器人模型到 Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'g60pro',
            '-topic', 'robot_description',
            '-timeout', '60'
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        gazebo,
        gazebo_client,
        description_launch,
        spawn_robot
    ])
