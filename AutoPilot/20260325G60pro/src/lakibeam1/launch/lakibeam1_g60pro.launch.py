#!/usr/bin/env python3
"""
G60Pro 双 LakiBeam1S 单线激光雷达启动文件
右前雷达：192.168.2.151 → /lidar/single_1/scan (frame: single_lidar_1_link)
左后雷达：192.168.2.150 → /lidar/single_2/scan (frame: single_lidar_2_link)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 右前单线雷达（IP 192.168.2.151）→ frame: single_lidar_1_link → 话题: /lidar/single_1/scan
    # UDP 目标端口 2369（需与雷达 Web 配置一致）
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
            'inverted': True,  # 雷达物理倒装，驱动层翻转数据
            'angle_offset': 0,
            'scanfreq': '30',
            'filter': '3',
            'laser_enable': 'true',
            'scan_range_start': '55',
            'scan_range_stop': '305',
        }]
    )

    # 左后单线雷达（IP 192.168.2.150）→ frame: single_lidar_2_link → 话题: /lidar/single_2/scan
    # UDP 目标端口 2368（需与雷达 Web 配置一致）
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
            'inverted': True,  # 雷达物理倒装，驱动层翻转数据
            'angle_offset': 0,
            'scanfreq': '30',
            'filter': '3',
            'laser_enable': 'true',
            'scan_range_start': '55',
            'scan_range_stop': '305',
        }]
    )

    return LaunchDescription([
        front_right_node,
        rear_left_node,
    ])