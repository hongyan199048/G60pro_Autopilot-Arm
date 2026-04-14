#!/usr/bin/env python3
"""
arm_vision 启动文件

同时启动粗位姿节点（vision_node）和 ICP 精定位节点（icp_node）。

用法：
    ros2 launch arm_vision arm_vision.launch.py

前置条件：
    - yolo_env conda 环境已激活（launch 中通过 executable 路径指定）
    - Orbbec DaBai DCW2 相机已连接
    - ROS2 工作空间已 source
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def get_vision_dir():
    """获取 Vision 根目录（arm_vision 的祖父目录）。"""
    # arm_vision/arm_vision/launch/__file__ → arm_vision/ → Vision/
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def generate_launch_description():
    vision_dir = get_vision_dir()
    orbbec_lib_dir = os.path.join(vision_dir, 'pyorbbecsdk', 'install', 'lib')

    # 设置环境变量
    env_vars = [
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            f'{orbbec_lib_dir}:$LD_LIBRARY_PATH'
        ),
        SetEnvironmentVariable(
            'PYTHONPATH',
            f'{orbbec_lib_dir}:$PYTHONPATH'
        ),
    ]

    # 节点可执行文件路径（yolo_env conda 环境中的 Python）
    python_exe = '/home/admin123/miniconda3/envs/yolo_env/bin/python3'

    # 粗位姿节点
    vision_node = Node(
        package='arm_vision',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{
            'yolo_model': os.path.join(vision_dir, 'detect/dc_charging_v2i/weights/best.pt'),
            'confidence': 0.5,
            'ransac_iterations': 100,
            'ransac_threshold_m': 0.005,
            'publish_rate': 10.0,
        }],
        # 强制使用 yolo_env 中的 Python（确保所有依赖可用）
        emulate_tty=True,
    )

    # ICP 精定位节点
    icp_node = Node(
        package='arm_vision',
        executable='icp_node',
        name='icp_node',
        output='screen',
        parameters=[{
            'yolo_model': os.path.join(vision_dir, 'detect/dc_charging_v2i/weights/best.pt'),
            'template_path': os.path.join(
                vision_dir,
                'datasets/直流充电座STP和点云/charging_port_template.pcd'
            ),
            'confidence': 0.5,
            'voxel_size': 0.002,
            'icp_max_dist': 0.010,
            'icp_max_iter': 50,
            'publish_rate': 10.0,
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        *env_vars,
        vision_node,
        icp_node,
    ])
