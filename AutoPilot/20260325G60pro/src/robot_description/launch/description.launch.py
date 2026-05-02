#!/usr/bin/env python3
"""
G60Pro 机器人描述启动文件
============================================================
功能：
1. 加载 URDF/XACRO 机器人模型（几何、物理、传感器）
2. 启动 robot_state_publisher 节点，发布静态 TF 树
3. 支持切换雷达类型（RoboSense Helios16 / Livox Mid360）

TF 树结构：
  base_footprint (地面投影，Z=0)
    └─ base_link (机器人几何中心)
        ├─ rs16_link (Helios16 雷达)
        ├─ imu_link (IMU)
        ├─ camera_*_link (4 路 RGBD 相机)
        └─ wheel_*_link (4 个轮子)
============================================================
"""

# ========== 导入模块 ==========
import os
from ament_index_python.packages import get_package_share_directory  # 获取 ROS2 包的安装路径
from launch import LaunchDescription              # Launch 文件主容器
from launch.actions import DeclareLaunchArgument  # 声明可配置参数
from launch.substitutions import LaunchConfiguration  # 获取参数值（运行时）
from launch_ros.actions import Node               # 定义 ROS2 节点
from xacro import process_file                    # 处理 XACRO 文件（宏展开）


def generate_launch_description():
    """
    Launch 文件入口函数
    返回：LaunchDescription 对象，包含节点和参数
    """

    # ========== 第 1 步：获取包的安装路径 ==========
    # get_package_share_directory：查找包的 share 目录
    # 路径示例：/home/admin123/.../install/robot_description/share/robot_description
    pkg_name = 'robot_description'
    pkg_dir = get_package_share_directory(pkg_name)

    # ========== 第 2 步：构建 XACRO 文件路径 ==========
    # XACRO：XML Macro，支持变量、条件、循环的 URDF 扩展格式
    # g60pro.urdf.xacro：机器人模型主文件，包含底盘、传感器、轮子等
    xacro_file = os.path.join(pkg_dir, 'urdf', 'g60pro.urdf.xacro')
    # 完整路径：.../install/robot_description/share/robot_description/urdf/g60pro.urdf.xacro

    # ========== 第 3 步：声明启动参数 ==========
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间（Gazebo 运行时设为 true）'
    )

    lidar_type_arg = DeclareLaunchArgument(
        'lidar_type',
        default_value='robosense_16',
        description='激光雷达类型: robosense_16 或 livox_mid360'
    )

    # ========== 第 4 步：处理 XACRO 文件 ==========
    # process_file：将 XACRO 宏展开为标准 URDF XML
    # mappings：传递给 XACRO 的参数字典（类似 C 的 #define）
    # TODO：此处硬编码为 'robosense_16'，应改为 LaunchConfiguration('lidar_type')
    doc = process_file(xacro_file, mappings={'lidar_type': 'robosense_16'})
    robot_description_content = doc.toxml()  # 转换为 XML 字符串
    # robot_description_content：完整的 URDF XML，包含所有 <link> 和 <joint>

    # ========== 第 5 步：定义 robot_state_publisher 节点 ==========
    # 功能：
    # 1. 解析 URDF 模型，提取所有 <joint> 的父子关系
    # 2. 发布静态 TF 变换（fixed joint）到 /tf_static
    # 3. 订阅 /joint_states，发布动态 TF（revolute/prismatic joint）
    robot_state_publisher_node = Node(
        package='robot_state_publisher',      # 系统包（ros-humble-robot-state-publisher）
        executable='robot_state_publisher',   # 可执行文件名
        name='robot_state_publisher',         # 节点名（ros2 node list 可见）
        output='screen',                      # 输出到终端

        # 节点参数
        parameters=[{
            # 参数 1：机器人模型（URDF XML 字符串）
            # robot_state_publisher 会解析此 XML，提取 TF 关系
            'robot_description': robot_description_content,

            # 参数 2：时间系统
            # True：使用仿真时间（Gazebo 提供的 /clock）
            # False：使用系统时间（实车）
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    # 节点启动后会发布的 TF：
    # - /tf_static：base_footprint → base_link → rs16_link, imu_link, camera_*_link, wheel_*_link
    # - /tf：如果有 /joint_states 输入，会发布动态关节的 TF（本项目无动态关节）

    # ========== 第 6 步：返回 LaunchDescription ==========
    return LaunchDescription([
        use_sim_time_arg,
        lidar_type_arg,
        robot_state_publisher_node
    ])

# ============================================================
# 使用示例
# ============================================================
# 1. 默认启动（RoboSense Helios16）：
#    ros2 launch robot_description description.launch.py
#
# 2. 切换雷达类型（需修复 TODO）：
#    ros2 launch robot_description description.launch.py lidar_type:=livox_mid360
#
# 3. 查看发布的 TF：
#    ros2 run tf2_tools view_frames
#    evince frames_*.pdf
#
# 4. 查看 URDF 模型：
#    ros2 run robot_state_publisher robot_state_publisher \
#      --ros-args -p robot_description:="$(xacro /path/to/g60pro.urdf.xacro)"
#    rviz2  # 添加 RobotModel 显示项
#
# 5. 查看节点信息：
#    ros2 node info /robot_state_publisher
#    ros2 param list /robot_state_publisher

# ============================================================
# 已知问题（TODO）
# ============================================================
# 1. lidar_type 参数未实际使用：
#    - 声明了 lidar_type_arg，但 process_file 中硬编码为 'robosense_16'
#    - 修复方法：改为 mappings={'lidar_type': LaunchConfiguration('lidar_type')}
#    - 注意：LaunchConfiguration 返回的是 Substitution 对象，需要在运行时解析
#
# 2. use_sim_time 硬编码为 True：
#    - 实车启动时应该是 False
#    - 修复方法：添加 use_sim_time 参数，传递给 robot_state_publisher
#
# 3. XACRO 处理时机：
#    - 当前在 launch 文件加载时处理（静态）
#    - 如果需要动态切换雷达，需要改为运行时处理

# ============================================================
# 与其他节点的关系
# ============================================================
# 上游节点（提供数据）：
#   - 无（robot_state_publisher 只读取 URDF，不订阅话题）
#
# 下游节点（使用 TF）：
#   - cartographer_node：需要 base_link → rs16_link 的 TF
#   - nav2_controller：需要 map → base_footprint 的 TF
#   - rviz2：需要完整 TF 树来显示机器人模型
#
# 并行节点：
#   - rslidar_sdk_node：发布点云（frame_id: rs16_link）
#   - cartographer_node：发布动态 TF（map → base_footprint）
#
# TF 树分工：
#   - robot_state_publisher：发布静态 TF（base_footprint 以下）
#   - cartographer_node：发布动态 TF（map → base_footprint）
#   - 完整链路：map → base_footprint → base_link → rs16_link