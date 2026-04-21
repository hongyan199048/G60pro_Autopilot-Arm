#!/usr/bin/env python3
"""
G60Pro Nav2 导航启动文件 — 无 AMCL 版（Cartographer 纯定位）

slam=False 时 Nav2 bringup 强制启动 map_server + amcl，
本文件替代 bringup_launch.py 的 localization 部分：
  - 启动 map_server（加载 .yaml 静态地图，供全局代价地图使用）
  - 启动导航节点（controller/planner/bt_navigator/smoother/behavior/...）
  - 不启动 AMCL

定位完全由 Cartographer 提供（map→base_footprint）。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    robot_navigation_share = get_package_share_directory('robot_navigation')
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # ── 启动参数 ──────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')

    # ── 参数重写（yaml_filename = map yaml 路径）───
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml_file,
            },
            convert_types=True),
        allow_substs=True)

    # 所有节点（不含 AMCL）
    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ── 声明启动参数 ──────────────────────────────
    declare_map_yaml = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(robot_navigation_share, 'config', 'nav2_params_real.yaml'),
        description='Nav2 参数文件')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true')
    declare_respawn = DeclareLaunchArgument(
        'use_respawn', default_value='False')
    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info')

    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # ── 节点列表 ──────────────────────────────────
    # pointcloud_to_laserscan 在 navigation_real.launch.py 中启动，
    # 这里只包含 Nav2 核心节点
    # yaml_filename 直接在 Node 参数中传入（绕过 RewrittenYaml 的嵌套替换不确定性）
    map_server_params = [{'yaml_filename': map_yaml_file, 'use_sim_time': use_sim_time}]

    nodes = [
        # map_server（加载静态地图，供全局代价地图）
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=map_server_params,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        # controller_server
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),

        # smoother_server
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        # planner_server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        # behavior_server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        # bt_navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        # waypoint_follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        # velocity_smoother
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel')
            ]),

        # lifecycle_manager（统一管理所有节点的生命周期）
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }]),
    ]

    # ── 启动描述 ──────────────────────────────────
    ld = LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        declare_map_yaml,
        declare_params,
        declare_sim_time,
        declare_autostart,
        declare_respawn,
        declare_log_level,
        GroupAction(nodes),
    ])
    return ld