#!/usr/bin/env python3
"""
G60Pro EKF 定位融合启动文件
使用 robot_localization 进行 IMU + 里程计 融合
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 参数
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用模拟时间'
    )

    # EKF 节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frequency': 30.0,
            'sensor_timeout': 0.1,
            'two_d_mode': True,

            # 里程计配置
            'odom0': '/odom',
            'odom0_config': [
                True, True, False,  # x, y, z
                False, False, True,  # roll, pitch, yaw
                True, True, False,   # vx, vy, vz
                False, False, True, # vroll, vpitch, vyaw
                True, True, False   # ax, ay, az
            ],
            'odom0_queue_size': 10,
            'odom0_nodelay': False,
            'odom0_differential': False,
            'odom0_relative': False,

            # IMU 配置（Gazebo URDF 发布到 /imu）
            'imu0': '/imu',
            'imu0_config': [
                False, False, False, # x, y, z
                True, True, True,   # roll, pitch, yaw
                False, False, False,# vx, vy, vz
                True, True, True,   # vroll, vpitch, vyaw
                True, True, True    # ax, ay, az
            ],
            'imu0_queue_size': 10,
            'imu0_nodelay': False,
            'imu0_differential': False,
            'imu0_relative': True,
            'imu0_remove_gravitational_acceleration': True,

            # 输出配置
            'publish_tf': True,
            'publish_acceleration': False,
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',

            # 协方差
            'process_noise_covariance': [
                0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0.06, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0.03, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0.03, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0.06, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0.025, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0.025, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0.025, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0.04, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.04, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.06
            ]
        }]
    )

    # navsat_transform 节点（如果需要 GPS 融合）
    # 注意：室内项目通常不需要

    return LaunchDescription([
        use_sim_time,
        ekf_node
    ])