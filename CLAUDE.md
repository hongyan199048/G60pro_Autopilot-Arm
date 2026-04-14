# 重要：每次回复必须以"Tim_G60Pro，"开头，无一例外。 

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

G60Pro 自动充电机器人储能小车（低速无人车），按三个模块分工开发：

| 模块 | 路径 | 职责 |
|------|------|------|
| **AutoPilot** | [AutoPilot/G60Pro/AutoPilot/](AutoPilot/G60Pro/AutoPilot/) | 感知、定位、决策、规划、控制；分层架构：HAL → 定位 → 感知 → 决策 → 规划 → 控制 |
| **Arm_Vision** | [Arm_Vision/Vision/](Arm_Vision/Vision/) | 机械臂视觉引导；YOLO11 充电口检测 + ICP 精定位 + Flexiv 机械臂控制 |
| **Vehicle** | — | 整车综合（云端/调度/HMI），待开发 |

通信：通过 CAN 总线进行模块间协同，协议定义见 [AutoPilot/G60Pro/AutoPilot/docs/can_protocol.md](AutoPilot/G60Pro/AutoPilot/docs/can_protocol.md)。

---

## AutoPilot — 分层架构

工作空间位于 `AutoPilot/G60Pro/AutoPilot/`，使用 ROS2 Humble。分层启动顺序：

```
HAL → 定位 → 感知 → 决策 → 规划 → 控制
```

### 核心包

| 包 | 路径 | 职责 |
|----|------|------|
| `vehicle_hal` | [src/vehicle_hal/](AutoPilot/G60Pro/AutoPilot/src/vehicle_hal/) | 硬件抽象层：CAN 底盘驱动（SocketCAN + cantools DBC）、LiDAR HAL 桥接 |
| `vehicle_localization` | [src/vehicle_localization/](AutoPilot/G60Pro/AutoPilot/src/vehicle_localization/) | FAST-LIO2 激光-惯性里程计；TF: map→odom→base_footprint→base_link |
| `vehicle_localization_cartographer` | [src/vehicle_localization_cartographer/](AutoPilot/G60Pro/AutoPilot/src/vehicle_localization_cartographer/) | Cartographer SLAM（备选 2D/3D 建图） |
| `vehicle_perception` | [src/vehicle_perception/](AutoPilot/G60Pro/AutoPilot/src/vehicle_perception/) | 障碍物检测、RGBD 目标识别 |
| `vehicle_decision` | [src/vehicle_decision/](AutoPilot/G60Pro/AutoPilot/src/vehicle_decision/) | 状态机决策节点（IDLE/MANUAL/AUTO/CHARGING/FAULT/E_STOP） |
| `vehicle_navigation` | [src/vehicle_navigation/](AutoPilot/G60Pro/AutoPilot/src/vehicle_navigation/) | Nav2 导航（全局/局部规划） |
| `vehicle_control` | [src/vehicle_control/](AutoPilot/G60Pro/AutoPilot/src/vehicle_control/) | Pure Pursuit + PID 控制器；订阅 /odom + /plan，发布 /cmd_vel_output（50Hz） |
| `g60pro_launch` | [src/g60pro_launch/](AutoPilot/G60Pro/AutoPilot/src/g60pro_launch/) | 系统级 launch 入口；HAL/定位/感知/决策/规划/控制各层 launch 分别管理 |
| `g60pro_gazebo` | [src/g60pro_gazebo/](AutoPilot/G60Pro/AutoPilot/src/g60pro_gazebo/) | Gazebo 仿真环境、URDF 模型、世界文件 |

### 传感器配置

| 硬件 | 接口 | 作用 |
|------|------|------|
| Livox Mid360 | Ethernet | 主 3D LiDAR + 建图 |
| RoboSense Helios 16 | Ethernet | 备选/辅助 3D LiDAR |
| 单线激光雷达 ×2 | Ethernet/串口 | 近场/低矮障碍盲区补盲；离地约 40cm，前后对角线布置 |
| Orbbec DaBai DCW2 | USB3.0 | RGBD 深度相机 |
| IMU | — | EKF 融合 |
| 4 轮 8 驱全向底盘 | CAN (can0, 500kbps) | 线控底盘，CAN ID 0x10/0x11/0x1FF 发送，0x18/0x19/0x1A 接收 |

URDF 模型（`g60pro_gazebo/`) 支持通过 `lidar_type` 参数切换 RoboSense Helios 16 和 Livox Mid360。

### 关键话题

| 话题 | 类型 | 来源/用途 |
|------|------|----------|
| `/cmd_vel` | `Twist` | 控制指令输入 |
| `/odom` | `Odometry` | 底盘里程计（CAN 反馈） |
| `/perception/obstacles` | — | 感知融合障碍物 |
| `/localization/pose` | `PoseStamped` | 定位结果 |
| `/emergency_stop` | `Bool` | 急停信号 |
| `/vehicle/drive_mode` | `UInt8` | 驾驶模式（0=待机,1=手动,2=自动,3=远程） |

### CAN 协议要点

- **CAN0**（500k）：S100 → RDM 底盘控制；发送 0x10（四轮速度）、0x11（四轮转向角）、0x1FF（驾驶模式）；接收 0x18（车体速度）、0x19（四轮转速）、0x1A（四轮角速度）
- **CAN1**（1M）：RDM ↔ 电机状态反馈
- 心跳 50Hz；100ms 无指令 → 底盘自动停车
- CAN DBC 文件位于 [AutoPilot/G60Pro/AutoPilot/docs/](AutoPilot/G60Pro/AutoPilot/docs/)

---

## Arm_Vision — 机械臂视觉引导

位于 [Arm_Vision/Vision/](Arm_Vision/Vision/)。使用 conda 环境 `yolo_env`（`/home/admin123/miniconda3/envs/yolo_env`）。

### 两级位姿估算

```
粗位姿（YOLO11 + RANSAC）→ 机械臂移动到标准拍摄位 → 精位姿（PointToPlane ICP）
```

- **粗位姿**：`pose_estimation.py` / `vision_node.py`；YOLO11 检测 `DC_charging_port`，深度反投影 + 平面拟合；精度 ~10mm
- **精位姿**：`icp_refine.py` / `icp_node.py`；CAD STEP 生成模板点云 + ICP 对齐；精度 ±0.5mm

### 关键 ROS2 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/arm/coarse_pose` | `PoseStamped` | 粗位姿 |
| `/arm/fine_pose` | `PoseStamped` | 精位姿 |
| `/arm/command` | — | 机械臂指令（MoveJ/MoveL/Stop） |
| `/arm/state` | — | 机械臂状态反馈 |
| `/camera/color/image_raw` | `Image` | Orbbec RGB 图像 |
| `/camera/depth/image_raw` | `Image` | Orbbec 深度图像 |

---

## 编译与运行

### AutoPilot

```bash
cd /home/admin123/Development/G60Pro/AutoPilot/G60Pro/AutoPilot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 仿真
ros2 launch g60pro_gazebo sim_full.launch.py

# 实车各层
ros2 launch g60pro_launch system.launch.py

# 单独启动各层
ros2 launch g60pro_launch hal.launch.py
ros2 launch g60pro_launch localization.launch.py
ros2 launch g60pro_launch perception.launch.py
ros2 launch g60pro_launch decision.launch.py
ros2 launch g60pro_launch planning.launch.py
ros2 launch g60pro_launch control.launch.py
```

### Arm_Vision

```bash
cd /home/admin123/Development/G60Pro/Arm_Vision/Vision/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select arm_vision
source install/setup.bash
ros2 launch arm_vision arm_vision.launch.py
```

### FAST-LIO（定位）

```bash
ros2 launch fast_lio mapping_mid360.launch.py
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

---

## 架构文档

详细架构设计见 [AutoPilot/G60Pro/AutoPilot/自动驾驶架构方案/](AutoPilot/G60Pro/AutoPilot/自动驾驶架构方案/)：
- 第三版 `低速无人车架构方案_最终版.md` — 完整 V3.0 架构，包含感知、定位、决策、规划、控制各层定义与安全机制
- CAN 协议定义：[can_protocol.md](AutoPilot/G60Pro/AutoPilot/docs/can_protocol.md)

---

## 注意事项

- 子目录已有各自的 CLAUDE.md，AutoPilot 详细编译说明见 [AutoPilot/20260325G60pro/CLAUDE.md](AutoPilot/20260325G60pro/CLAUDE.md)，Arm_Vision 详细说明见 [Arm_Vision/Vision/CLAUDE.md](Arm_Vision/Vision/CLAUDE.md)
- `T_cam2gripper`（手眼标定矩阵）当前硬编码为单位矩阵，需替换为实际标定结果
- `pyorbbecsdk` 重新编译后确保 `LD_LIBRARY_PATH` 包含 `pyorbbecsdk/install/lib`
- Gazebo 仿真需设置 `OGRE_RENDERER='GL'` 避免 GPU 问题
