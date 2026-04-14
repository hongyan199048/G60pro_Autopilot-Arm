#!/usr/bin/env python3
"""
G60Pro 传感器启动文件
启动所有传感器驱动
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch
import os
import os


def generate_launch_description():
    # 参数声明
    enable_lidar16 = DeclareLaunchArgument(
        'enable_lidar16',
        default_value='true',
        description='启用16线激光雷达'
    )

    enable_single_lidar = DeclareLaunchArgument(
        'enable_single_lidar',
        default_value='true',
        description='启用单线激光雷达'
    )

    enable_orbbec = DeclareLaunchArgument(
        'enable_orbbec',
        default_value='true',
        description='启用Orbbec多相机'
    )

    enable_imu = DeclareLaunchArgument(
        'enable_imu',
        default_value='true',
        description='启用IMU'
    )

    # 传感器管理节点
    sensors_node = Node(
        package='robot_sensors',
        executable='sensors_node',
        name='robot_sensors',
        output='screen'
    )

    # 16线激光雷达节点 (Helios16)
    # 注意：需要根据实际 SDK 配置
    lidar16_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar',
        output='screen',
        condition=launch.condition.IfCondition(LaunchConfiguration('enable_lidar16'))
    )

    # 单线激光雷达节点 (LakiBeam1s)
    single_lidar_front_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_node',
        name='single_lidar_front',
        output='screen',
        condition=launch.condition.IfCondition(LaunchConfiguration('enable_single_lidar'))
    )

    single_lidar_rear_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_node',
        name='single_lidar_rear',
        output='screen',
        condition=launch.condition.IfCondition(LaunchConfiguration('enable_single_lidar'))
    )

    # Orbbec 多相机节点 (DaBai DCW2)
    # 需要根据实际 USB 端口配置 usb_port 参数
    # 4个相机默认配置为前/后/左/右
    orbbec_front_node = Node(
        package='orbbec_camera',
        executable='orbbec_camera_node',
        name='orbbec_camera_front',
        namespace='orbbec',
        output='screen',
        parameters=[{
            'camera_name': 'front',
            'usb_port': '1-1',  # 需根据实际USB端口修改
            'device_num': 1,
            'enable_depth': True,
            'enable_color': True,
            'depth_width': 640,
            'depth_height': 400,
            'color_width': 640,
            'color_height': 480,
            'depth_fps': 10,
            'color_fps': 10,
        }],
        condition=launch.condition.IfCondition(LaunchConfiguration('enable_orbbec'))
    )

    orbbec_rear_node = Node(
        package='orbbec_camera',
        executable='orbbec_camera_node',
        name='orbbec_camera_rear',
        namespace='orbbec',
        output='screen',
        parameters=[{
            'camera_name': 'rear',
            'usb_port': '2-1',  # 需根据实际USB端口修改
            'device_num': 1,
            'enable_depth': True,
            'enable_color': True,
            'depth_width': 640,
            'depth_height': 400,
            'color_width': 640,
            'color_height': 480,
            'depth_fps': 10,
            'color_fps': 10,
        }],
        condition=launch.condition.IfCondition(LaunchConfiguration('enable_orbbec'))
    )

    orbbec_left_node = Node(
        package='orbbec_camera',
        executable='orbbec_camera_node',
        name='orbbec_camera_left',
        namespace='orbbec',
        output='screen',
        parameters=[{
            'camera_name': 'left',
            'usb_port': '3-1',  # 需根据实际USB端口修改
            'device_num': 1,
            'enable_depth': True,
            'enable_color': True,
            'depth_width': 640,
            'depth_height': 400,
            'color_width': 640,
            'color_height': 480,
            'depth_fps': 10,
            'color_fps': 10,
        }],
        condition=launch.condition.IfCondition(LaunchConfiguration('enable_orbbec'))
    )

    orbbec_right_node = Node(
        package='orbbec_camera',
        executable='orbbec_camera_node',
        name='orbbec_camera_right',
        namespace='orbbec',
        output='screen',
        parameters=[{
            'camera_name': 'right',
            'usb_port': '4-1',  # 需根据实际USB端口修改
            'device_num': 1,
            'enable_depth': True,
            'enable_color': True,
            'depth_width': 640,
            'depth_height': 400,
            'color_width': 640,
            'color_height': 480,
            'depth_fps': 10,
            'color_fps': 10,
        }],
        condition=launch.condition.IfCondition(LaunchConfiguration('enable_orbbec'))
    )

    # IMU 节点
    imu_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_filter',
        output='screen',
        condition=launch.condition.IfCondition(LaunchConfiguration('enable_imu'))
    )

    return LaunchDescription([
        enable_lidar16,
        enable_single_lidar,
        enable_orbbec,
        enable_imu,
        sensors_node,
        # 实际传感器节点（根据硬件启用）
        # lidar16_node,
        # single_lidar_front_node,
        # single_lidar_rear_node,
        orbbec_front_node,
        orbbec_rear_node,
        orbbec_left_node,
        orbbec_right_node,
        # imu_node
    ])