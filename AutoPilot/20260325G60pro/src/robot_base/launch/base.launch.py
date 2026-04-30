#!/usr/bin/env python3
"""
G60Pro 底盘控制启动文件
功能：启动底盘运动学控制节点，将 /cmd_vel 转换为电机指令 /motor_cmd
"""

# Launch 系统核心模块
from launch import LaunchDescription              # Launch 文件主容器
from launch.actions import DeclareLaunchArgument  # 声明可配置参数
from launch.substitutions import LaunchConfiguration  # 获取参数值
from launch_ros.actions import Node               # 定义 ROS2 节点


def generate_launch_description():
    """
    Launch 文件入口函数，返回 LaunchDescription 对象
    """

    # ========== 第 1 步：声明启动参数（可通过命令行覆盖） ==========

    # 参数 1：ROS2 时间系统开关
    # - false（默认）：使用系统时间，适用于实车
    # - true：使用仿真时间，适用于 Gazebo
    # 命令行覆盖：ros2 launch robot_base base.launch.py use_sim_time:=true
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间'
    )

    # 参数 2：业务逻辑模拟开关
    # - false（默认）：发送真实 CAN 指令到底盘
    # - true：不发送 CAN，只发布调试话题（用于无硬件调试）
    # 注意：与 use_sim_time 独立，可以 use_sim_time=false 但 use_sim=true
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='模拟模式'
    )

    # 参数 3：最大线速度限制（m/s）
    # 用途：限制 cmd_vel.linear.x 和 cmd_vel.linear.y，防止超速
    # 安全考虑：充电对接时可降低到 0.3 m/s
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='1.0',
        description='最大线速度 (m/s)'
    )

    # 参数 4：最大角速度限制（rad/s）
    # 用途：限制 cmd_vel.angular.z，防止急转弯
    # 1.0 rad/s ≈ 57°/s，适合低速场景
    max_angular_arg = DeclareLaunchArgument(
        'max_angular',
        default_value='1.0',
        description='最大角速度 (rad/s)'
    )

    # ========== 第 2 步：定义底盘控制节点 ==========

    base_node = Node(
        package='robot_base',           # 包名（在 package.xml 中声明）
        executable='robot_base_node',   # 可执行文件名（在 CMakeLists.txt 中定义）
        name='robot_base',              # 节点运行时名称（ros2 node list 可见）
        output='screen',                # 输出到终端（也可用 'log' 写入日志）
        parameters=[{
            # ROS2 时间参数（从启动参数获取）
            'use_sim_time': LaunchConfiguration('use_sim_time'),

            # 底盘物理参数（硬编码，与实际硬件匹配）
            'wheel_radius': 0.15,    # 轮子半径 15cm，用于速度转换 v = ω × r
            'wheelbase_x': 0.75,     # 前后轮距 75cm，用于全向轮运动学逆解
            'wheelbase_y': 0.5,      # 左右轮距 50cm，用于全向轮运动学逆解

            # 速度限制参数（从启动参数获取）
            'max_velocity': LaunchConfiguration('max_velocity'),
            'max_angular': LaunchConfiguration('max_angular'),

            # 模拟模式开关（从启动参数获取）
            'use_sim': LaunchConfiguration('use_sim')
        }]
    )
    # 节点功能：
    # - 订阅 /cmd_vel (geometry_msgs/Twist)：接收速度指令
    # - 发布 /motor_cmd (robot_msgs/MotorCmd)：发送电机指令
    # - 订阅 /motor_state (robot_msgs/MotorState)：接收电机反馈
    # - 发布 /odom (nav_msgs/Odometry)：发布里程计
    # - 发布 TF：odom → base_footprint

    # ========== 第 3 步：返回 LaunchDescription ==========

    return LaunchDescription([
        use_sim_time_arg,   # 参数声明 1
        use_sim_arg,        # 参数声明 2
        max_velocity_arg,   # 参数声明 3
        max_angular_arg,    # 参数声明 4
        base_node           # 底盘节点
    ])

# ========== 使用示例 ==========
# 1. 实车模式（默认）：
#    ros2 launch robot_base base.launch.py
#
# 2. 仿真模式：
#    ros2 launch robot_base base.launch.py use_sim_time:=true use_sim:=true
#
# 3. 限速模式（安全测试）：
#    ros2 launch robot_base base.launch.py max_velocity:=0.3 max_angular:=0.5
#
# 4. 查看节点信息：
#    ros2 node info /robot_base
#    ros2 param list /robot_base