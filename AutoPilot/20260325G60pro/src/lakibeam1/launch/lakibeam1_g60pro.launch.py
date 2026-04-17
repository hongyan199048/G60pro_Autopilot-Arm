#!/usr/bin/env python3
"""
G60Pro 双 LakiBeam1S 单线激光雷达启动文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 右前单线雷达（IP 192.168.2.151）→ frame: single_lidar_1_link → 话题: /lidar/single_1/scan（橙色）
    # 注意：该雷达实际配置的 UDP 目标端口为 2369，需与 launch 中绑定的端口一致
    front_right_node = Node(
        package='lakibeam1',
        name='lakibeam1_front_right',
        executable='lakibeam1_scan_node',
        output='screen',
        parameters=[{
            'frame_id': 'single_lidar_1_link',
            'output_topic': '/lidar/single_1/scan',
            'hostip': '0.0.0.0',
            'port': '2369',
            'sensorip': '192.168.2.151',
            'inverted': False,
            'angle_offset': 0,
            'scanfreq': '30',
            'filter': '3',
            'laser_enable': 'true',
            'scan_range_start': '45',
            'scan_range_stop': '315',
        }]
    )

    # 左后单线雷达（IP 192.168.2.150）→ frame: single_lidar_2_link → 话题: /lidar/single_2/scan（绿色）
    # 注意：该雷达实际配置的 UDP 目标端口为 2368，需与 launch 中绑定的端口一致
    rear_left_node = Node(
        package='lakibeam1',
        name='lakibeam1_rear_left',
        executable='lakibeam1_scan_node',
        output='screen',
        parameters=[{
            'frame_id': 'single_lidar_2_link',
            'output_topic': '/lidar/single_2/scan',
            'hostip': '0.0.0.0',
            'port': '2368',
            'sensorip': '192.168.2.150',
            'inverted': False,
            'angle_offset': 0,
            'scanfreq': '30',
            'filter': '3',
            'laser_enable': 'true',
            'scan_range_start': '45',
            'scan_range_stop': '315',
        }]
    )

    return LaunchDescription([
        front_right_node,
        rear_left_node,
    ])
