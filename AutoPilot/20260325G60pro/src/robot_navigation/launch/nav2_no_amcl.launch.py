#!/usr/bin/env python3
"""
G60Pro Nav2 导航启动文件 — 无 AMCL 版（Cartographer 纯定位）

三种模式（互斥）：
  1. 纯定位：有 .pbstream 文件 → Cartographer 从文件加载地图做 scan matching
  2. 静态地图：有 .yaml 文件 → map_server 发布 /map
  3. 实时建图：无 .yaml 无 .pbstream → cartographer_occupancy_grid_node 发布 /map

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

    # ── 启动参数 ──────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    pbstream_file = LaunchConfiguration('pbstream_file')

    # ── 参数重写 ──────────────────────────────────
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
    # 注意：map_server 必须加入 lifecycle_nodes，否则 lifecycle_manager 不会给它发送 configure/activate，
    # 导致 map_server 停留在 unconfigured 状态，/map 永远不发布
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
        default_value='',
        description='静态地图 .yaml 路径（空=实时 /map）')
    declare_pbstream = DeclareLaunchArgument(
        'pbstream_file',
        default_value='',
        description='.pbstream 文件路径（纯定位模式）；非空则跳过所有 /map 发布者')
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

    # ── 条件判断 ────────────────────────────────
    have_map_yaml = PythonExpression(["'", LaunchConfiguration('map'), "' != ''"])
    have_pbstream = PythonExpression(["'", pbstream_file, "' != ''"])

    # ── 节点列表 ──────────────────────────────────
    nodes = [
        # cartographer_occupancy_grid_node（仅当无 .yaml 且无 .pbstream 时启动）
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': 0.05,
            }],
            arguments=['--ros-args', '--log-level', 'warn'],
            condition=IfCondition(PythonExpression(['not ', have_map_yaml, ' and not ', have_pbstream])),
        ),

        # map_server（仅当有 .yaml 且无 .pbstream 时启动）
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[{'yaml_filename': map_yaml_file, 'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            condition=IfCondition(PythonExpression([have_map_yaml, ' and not ', have_pbstream])),
        ),

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
        declare_pbstream,
        declare_params,
        declare_sim_time,
        declare_autostart,
        declare_respawn,
        declare_log_level,
        GroupAction(nodes),
    ])
    return ld