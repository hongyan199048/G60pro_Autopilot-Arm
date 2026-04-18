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

命名规则：`_sim` = Gazebo 仿真，`_real` = 实车，无后缀 = 通用

### 仿真环境（Gazebo）
```bash
cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro/docs
./start_slam_sim.sh        # Gazebo + Cartographer SLAM + RViz
./start_nav_sim.sh         # Gazebo SLAM 建图 → 保存地图 → Nav2 导航
```

### 实车环境
```bash
cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro/docs
./start_slam_real.sh       # Helios16 + Cartographer + RViz
./view_sensors_real.sh     # 查看所有传感器（Helios16 + 双 LakiBeam1S）
```

### 通用工具
```bash
cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro/docs
./save_map.sh              # 保存当前 /map 为 maps/g60pro_v*.pgm + .yaml
```

## 键盘控制
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/cmd_vel
```




## 系统架构

### 功能分层

| 层级 | 包 | 职责 |
|------|-----|------|
| 感知 | `robot_slam` / `rslidar_sdk` / `lakibeam1` / `orbbec_camera` | 激光雷达 + RGBD 相机 + IMU；Cartographer 2D SLAM |
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
| `robot_gazebo/launch/sim.launch.py` | Gazebo 仿真：Gazebo + 描述 + spawn |
| `robot_slam/launch/slam.launch.py` | Cartographer SLAM |
| `robot_navigation/launch/navigation.launch.py` | Nav2 导航（含 AMCL 定位） |
| `robot_base/launch/base.launch.py` | 底盘运动学节点 |
| `robot_description/launch/description.launch.py` | URDF + TF 发布 |

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

## 注意事项

- `robot_description/launch/description.launch.py` 中 `lidar_type` 参数硬编码为 `robosense_16`，如需切换雷达需修改该文件中的 `process_file` 调用。

## Orbbec 相机

使用 `orbbec_camera/launch/dabai_dcw2.launch.py` 启动，`usb_port` 参数区分设备，需根据实际 USB 端口修改。

## RViz 配置

所有 RViz 配置文件统一在 `robot_rviz/rviz/`：

| 文件 | 用途 |
|------|------|
| `slam.rviz` | Gazebo 仿真 SLAM 建图视图 |
| `slam_real.rviz` | 实车 SLAM 建图视图（Helios16 点云）|
| `navigation.rviz` | Nav2 导航视图（地图 + 路径 + 代价地图） |
| `sensors_view.rviz` | 实车传感器查看（Helios16 + 双 LakiBeam1S） |
