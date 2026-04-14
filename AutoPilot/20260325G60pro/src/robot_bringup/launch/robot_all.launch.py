#!/usr/bin/env python3
"""
G60Pro 整车启动文件
启动所有功能包：底盘、CAN、传感器、定位
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 参数
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用模拟时间'
    )

    # 1. 机器人描述 (URDF + TF)
    description_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('robot_description'), 'launch', 'description.launch.py'),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 2. 底盘控制
    base_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('robot_base'), 'launch', 'base.launch.py'),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 3. CAN 通信
    can_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('robot_can'), 'launch', 'can.launch.py'),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 4. 传感器 (可选)
    sensors_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('robot_sensors'), 'launch', 'sensors.launch.py'),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 5. 定位融合 (可选)
    localization_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('robot_localization'), 'launch', 'ekf.launch.py'),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 6. RViz2 (可选)
    rviz_config = os.path.join(get_package_share_directory('robot_bringup'), 'rviz', 'robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # 7. 键盘控制 (可选)
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e'
    )

    return LaunchDescription([
        use_sim_time,
        description_launch,
        base_launch,
        can_launch,
        sensors_launch,
        # localization_launch,
        # rviz_node,
        # teleop_node
    ])