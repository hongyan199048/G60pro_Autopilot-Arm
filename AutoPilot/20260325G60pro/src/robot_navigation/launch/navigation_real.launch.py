#!/usr/bin/env python3
"""
G60Pro Nav2 导航启动文件 — 实车（Helios16）
pointcloud_to_laserscan 高度参数与 cartographer_real.lua 对齐

定位由 Cartographer 提供（map→base_footprint），不启动 AMCL。
使用 nav2_no_amcl.launch.py 替代标准 bringup_launch.py。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用模拟时间（实车固定为 false）'
    )

    robot_navigation_share = get_package_share_directory('robot_navigation')
    nav2_launch_file = os.path.join(robot_navigation_share, 'launch', 'nav2_no_amcl.launch.py')

    map_file = DeclareLaunchArgument(
        'map',
        default_value='',
        description='地图文件路径（用于 map_server；空=使用 Cartographer 实时 /map）'
    )

    pbstream_file = DeclareLaunchArgument(
        'pbstream_file',
        default_value='',
        description='.pbstream 文件路径（纯定位模式）；非空则跳过所有 /map 发布者'
    )

    params_file = os.path.join(robot_navigation_share, 'config', 'nav2_params_real.yaml')

    # 3D 点云 → 2D 激光扫描，供代价地图使用
    # 关键修复：target_frame 必须是激光雷达自己的坐标系（rs16_link），不能是 base_footprint
    # 这样 obstacle_layer 才能正确计算障碍物的 3D 位置和高度
    # 高度范围（相对 rs16_link，激光雷达离地 1.54m）：
    #   min_height: -1.4m（过滤地面，地面在 -1.54m）
    #   max_height: 0.5m（过滤天花板，保留 0.1-2.0m 高度的障碍物）
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'target_frame': 'rs16_link',  # 修复：使用激光雷达坐标系，不是 base_footprint
            'transform_tolerance': 0.5,
            'min_height': -1.5,  # 更接近地面（地面在 -1.54m）
            'max_height': 1.0,   # 提高上限到 1.0m，检测更高障碍物
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00436,
            'scan_time': 0.1,
            'range_min': 0.1,  # 降低到 0.1m，检测近距离障碍物
            'range_max': 30.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/lidar/rs16/points'),
            ('scan', '/scan'),
        ],
    )

    # 4 路 RGBD 深度点云 → 2D 激光扫描（近场盲区补盲）
    # target_frame 用 camera_link（Z 向上），min/max_height 在 camera_link 的 Z 轴过滤
    # camera_link 离地约 1.15m，地面在 Z≈-1.15，障碍物在 Z≈-1.0 ~ 0.85
    camera_pcl_scanners = []
    camera_configs = [
        ('camera_front', '/camera_front/depth_registered/points', '/camera_front/depth/scan', 'camera_front_link'),
        ('camera_rear',  '/camera_rear/depth_registered/points',  '/camera_rear/depth/scan',  'camera_rear_link'),
        ('camera_left',  '/camera_left/depth_registered/points',  '/camera_left/depth/scan',  'camera_left_link'),
        ('camera_right', '/camera_right/depth_registered/points', '/camera_right/depth/scan', 'camera_right_link'),
    ]
    for cam_name, cloud_topic, scan_topic, target_frame in camera_configs:
        camera_pcl_scanners.append(Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name=f'pointcloud_to_laserscan_{cam_name}',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'target_frame': target_frame,
                'transform_tolerance': 0.5,
                'min_height': -1.2,   # 地面在 camera_link 约 -1.15m
                'max_height': 1.0,    # 检测约 2.15m 以下障碍物
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00873,  # ~1° 分辨率，减少计算量
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 5.0,     # RGBD 相机可靠范围 ~5m
                'use_inf': True,
            }],
            remappings=[
                ('cloud_in', cloud_topic),
                ('scan', scan_topic),
            ],
        ))

    # Nav2 导航节点（无 AMCL 版）
    # map_server 加载静态 .yaml 地图（供全局代价地图）
    # 所有定位 TF 由 Cartographer 提供（map→base_footprint）
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': params_file,
            'map': LaunchConfiguration('map'),
            'pbstream_file': LaunchConfiguration('pbstream_file'),
            'autostart': 'True'
        }.items()
    )

    # Initial Pose Relay 节点（处理 RViz 2D Pose Estimate）
    # 通过 finish_trajectory + start_trajectory 重置 Cartographer 定位
    cartographer_config_dir = get_package_share_directory('robot_slam')

    initial_pose_relay = Node(
        package='robot_navigation',
        executable='initial_pose_relay',
        name='initial_pose_relay',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'configuration_directory': cartographer_config_dir + '/config',
            'configuration_basename': 'cartographer_real_localization.lua',
        }],
    )

    return LaunchDescription([
        use_sim_time,
        map_file,
        pbstream_file,
        pointcloud_to_laserscan,
        *camera_pcl_scanners,
        nav2_launch,
        initial_pose_relay,
    ])
