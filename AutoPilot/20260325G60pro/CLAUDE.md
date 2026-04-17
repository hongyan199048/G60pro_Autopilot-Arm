# 重要：每次回复必须以"Tim_20260325G60pro，"开头，无一例外。 

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 编译

```bash
cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build
```

## 启动
### 虚拟环境启动slam
```bash
cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro/docs
./start_slam.sh
```
## 键盘控制
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/cmd_vel
```

### 实机启动slam
```bash
cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro/docs
./start_slam_real.sh
```
### rviz实机所有外设显示（orbbec相机暂时被注释）
```bash
cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro/docs
./view_sensors.sh
```
## 保存地图
```bash
cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro/docs
./save_map.sh
```
保存完成后会生成 .pgm + .yaml，Nav2 直接用




## 系统架构

### 功能分层

| 层级 | 包 | 职责 |
|------|-----|------|
| 感知 | `robot_sensors` / `robot_slam` | 激光雷达 + RGBD 相机 + IMU；Cartographer 2D SLAM |
| 定位 | `robot_localization` | robot_localization EKF 融合（odom + IMU → /odom_combined） |
| 规划 | `robot_navigation` | Nav2 全局/局部规划 |
| 控制 | `robot_base` / `robot_can` | 底盘运动学解算 + 双 CAN 总线通信 |
| 模型 | `robot_description` | URDF/XACRO 机器人模型 + TF 发布 |

### 数据流

```
/cmd_vel
  └─> robot_base_node  (运动学解算: Twist → 4转向角 + 4驱动速度)
       ├─> /motor_cmd  (robot_msgs/MotorCmd)
       │    └─> robot_can_node  (CAN1 500k → RDM 控制器)
       │         └─> CAN4 1M → 电机驱动
       └─> /odom  (里程计 + TF: odom → base_footprint)

传感器话题:
  /lidar/multi/points  ──> Cartographer ──> /map + /scan
  /lidar/single_1/scan ─┐
  /lidar/single_2/scan ─┴─> Nav2 避障
  /imu ──────────────────> Cartographer + EKF
  /camera/{front,rear,left,right}/* ──> 视觉感知
```

### 启动入口

| 启动文件 | 用途 |
|---------|------|
| `robot_bringup/launch/robot_all.launch.py` | 整车：描述 + 底盘 + CAN + 传感器 + 定位 |
| `robot_bringup/launch/robot_slam.launch.py` | SLAM 建图：描述 + 底盘 + 传感器 + Cartographer |
| `robot_bringup/launch/robot_navigation.launch.py` | 导航：描述 + 底盘 + 传感器 + EKF + Nav2 |
| `robot_gazebo/launch/sim.launch.py` | Gazebo 仿真：Gazebo + 描述 + spawn |
| `start_slam.sh` | 一键脚本：清理进程 → colcon build → Gazebo + 底盘 + SLAM + RViz |

### 关键话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `Twist` | 速度指令输入 |
| `/odom` | `Odometry` | 底盘里程计（gazebo_ros_planar_move / robot_base 发布） |
| `/motor_cmd` | `MotorCmd` | 4轮转向角 + 驱动速度 |
| `/motor_state` | `MotorState` | 电机状态反馈 |
| `/lidar/multi/points` | `PointCloud2` | 多线激光雷达点云 → remap → `points2` |
| `/lidar/single_1/scan` | `LaserScan` | 前侧单线雷达 |
| `/lidar/single_2/scan` | `LaserScan` | 后侧单线雷达 |
| `/imu` | `Imu` | IMU 数据 |

### TF 树

```
odom → base_footprint → base_link → (camera_*, imu_link, wheel_*, rs16_link/livox_link, single_lidar_*_link, manipulator_base_link)
```

仿真时 `odom → base_footprint` 由 `gazebo_ros_planar_move` 插件发布。实车时由 `robot_base_node` 发布。

## 机器人模型

URDF/XACRO：`robot_description/urdf/g60pro.urdf.xacro`

- 支持通过 `lidar_type` 参数切换多线雷达：`robosense_16`（默认）/ `livox_mid360`
- 4 轮 8 驱：每轮独立转向（steer_joint）+ 驱动（drive_joint），均为 `fixed` 类型
- 4 路 Orbbec RGBD 相机（前/后/左/右）+ 2 路单线雷达 + IMU

**注意**：`description.launch.py` 中 `lidar_type` 目前硬编码为 `robosense_16`，未读取 launch 参数。如需切换雷达，需修改 `description.launch.py` 中的 `process_file` 调用。

## CAN 通信

- CAN1（can0，500kbps）：发送控制指令到 RDM 控制器
- CAN4（can1，1Mbps）：接收电机状态反馈

DBC 协议文件位于 `robot_can/config/`。

## Gazebo 仿真关键配置

`sim.launch.py` 设置三个关键环境变量：
```bash
export OGRE_RENDERER='GL'              # 软渲染避免 GPU 问题
export GAZEBO_MODEL_DATABASE_URI=''    # 禁止从网络拉取模型，避免启动卡住
export GAZEBO_MODEL_PATH=...           # 包含 robot_description 和 /usr/share/gazebo-11/models
```

URDF 中所有 link 的 `inertial.mass` 设为 `0.001`，车体和机械臂的 `gravity` 设为 `0`。

## 已知 Bug

1. `robot_navigation/launch/navigation.launch.py` 缺少 `IncludeLaunchDescription` 和 `PythonLaunchDescriptionSource` 的 import，会导致启动失败。需在文件头部添加：
   ```python
   from launch.actions import IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   ```

2. `robot_description/launch/description.launch.py` 中 `lidar_type` 参数硬编码为 `robosense_16`，`lidar_type_arg` 声明了但未使用。

## Orbbec 相机

`sensors.launch.py` 中 4 个 Orbbec 相机使用 `usb_port` 参数区分设备，需根据实际 USB 端口修改。
