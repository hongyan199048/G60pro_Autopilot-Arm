
# G60Pro 自动驾驶小车 ROS2 完整开发文档
**硬件平台**：S100 RDK + 4轮8驱独立转向底盘 + Helios16多线雷达 + LakiBeam1s单线雷达 ×2 + DaBai DCW2-DW2 RGBD相机 ×4 + IMU + CAN + RDM 控制器
**执行器**：安普斯驱动电机 ×4 + 步科 Kinco 转向电机 ×4
**ROS 版本**：ROS2 Humble
**系统环境**：Ubuntu 22.04
**工程架构**：工业级标准 ROS2 工程
**适用场景**：室内外自动驾驶、SLAM 建图、导航避障、全向移动底盘控制
**CAN 总线支持**：CAN1/CAN4 双总线，适配双 DBC 协议

---

# 1. 工程概述
本工程为 **4轮8驱独立转向全向自动驾驶小车** 完整解决方案，包含：
- 底盘运动学解算（4转向+4驱动）
- CAN 总线硬件通信（S100→CAN1→RDM→CAN4→电机控制器，双 DBC）
- 多传感器驱动与数据融合
- EKF 定位融合（扩展卡尔曼滤波，多传感器数据融合）
- Cartographer 激光 SLAM 建图
- Nav2 全自动导航（全局规划 + DWA/MPC 局部规划）
- RViz 可视化、键盘控制、一键启动
- 标准化工程结构：src 内全部功能包 + robot_bringup 统一管理启动/配置/地图

---

# 2. 硬件系统配置（真实型号版）
## 2.1 底盘系统
- 类型：4轮独立转向 + 4轮独立驱动（8 电机）
- 轮子：左前(FL)、右前(FR)、左后(RL)、右后(RR)
- 车体尺寸：长 1.5m / 宽 1.0m / 高 1.0m
- 轮子半径：0.15m
- 转向角度：±90°
- 控制链路：S100 → CAN1 → RDM 控制器 → CAN4 → 电机控制器

## 2.2 传感器系统（真实型号）
- **多线激光雷达**：RoboSense Helios 16（16线）
- **单线激光雷达**：Richbeam LakiBeam 1s ×2（对角避障补盲）
- **RGBD相机**：DaBai DCW2-DW2 ×4（前/后/左/右全景感知）
- **IMU**：高性能姿态传感器

## 2.3 执行器系统（真实型号）
- **驱动电机**：安普斯 伺服/无刷驱动电机 ×4
- **转向电机**：步科 Kinco 高精度伺服转向电机 ×4
- 控制方式：CAN 总线控制（CAN4 DBC 协议）

## 2.4 控制链路
```
S100 RDK (ROS2 主控)
   ↓ CAN1 DBC 指令（500Kbps）
CAN1 总线
   ↓
RDM 控制器（协议转换）
   ↓ CAN4 DBC 指令（1Mbps）
CAN4 总线
   ↓
安普斯驱动电机 + 步科转向电机（共8台）
```

## 2.5 CAN 总线参数
| 总线 | 连接设备 | 波特率 | 终端电阻 |
|------|----------|--------|----------|
| CAN1 | S100 ↔ RDM | 500Kbps | 120Ω |
| CAN4 | RDM ↔ 电机 | 1Mbps | 120Ω |

---

# 3. 最终标准工程目录结构
```
robot_g60pro_ws/
├── src/
│   ├── robot_bringup           # 总启动包（含全部子文件夹）
│   │   ├── launch/            # 所有启动文件
│   │   ├── config/            # 导航/SLAM/定位/EKF 配置
│   │   ├── maps/              # 地图保存
│   │   ├── rviz/              # RViz 配置
│   │   ├── package.xml
│   │   └── setup.py
│   ├── robot_base              # 底盘解算
│   ├── robot_can               # CAN 通信 + DBC 解析
│   │   └── config/
│   │       ├── can1.dbc
│   │       └── can4.dbc
│   ├── robot_description       # URDF 模型
│   ├── robot_sensors           # 传感器驱动（雷达/相机/IMU）
│   ├── robot_localization      # EKF 定位融合
│   ├── robot_slam              # SLAM 建图
│   ├── robot_navigation        # Nav2 导航
│   └── robot_msgs               # 自定义消息
├── docs/                      # 开发文档
├── build/                     # 自动生成
├── install/                   # 自动生成
└── log/                       # 自动生成
```

---

# 4. 功能包说明
## 4.1 robot_bringup（全车总入口）
**内部标准子目录：**
- `launch/`：整车启动、SLAM、导航、传感器启动
- `config/`：Nav2、Cartographer、EKF、参数配置
- `maps/`：保存与加载地图
- `rviz/`：RViz 可视化配置

## 4.2 robot_base
订阅 `/cmd_vel` → 4轮8驱解算 → 输出 8 电机指令 `/motor_cmd`

## 4.3 robot_can
- 加载 can1.dbc / can4.dbc
- 发送指令到 CAN1 → RDM → CAN4 → 电机
- 解析电机状态并回传

## 4.4 robot_description
URDF + TF 坐标变换

## 4.5 robot_sensors
传感器驱动：
- Helios 16 多线雷达
- LakiBeam 1s 单线雷达 ×2
- DaBai DCW2-DW2 RGBD 相机 ×4
- IMU

## 4.6 robot_localization
EKF 融合：IMU + 轮速里程计 + 激光定位

## 4.7 robot_slam
Cartographer 激光建图（基于 Helios16 点云）

## 4.8 robot_navigation
Nav2 自动驾驶：
- 全局规划：找大路线
- 局部规划：DWA / MPC 避障控车

## 4.9 robot_msgs
适配 DBC 信号的自定义消息

---

# 5. 整车数据流
```
Nav2 (全局规划 + DWA/MPC)
   ↓ /cmd_vel
robot_base（8 电机解算）
   ↓ /motor_cmd
robot_can（封装 CAN1 DBC）
   ↓ CAN1 → RDM → CAN4（CAN4 DBC）
   ↓ 安普斯驱动电机 + 步科转向电机
   ↓ 电机状态回传
EKF 定位融合
   ↓
Nav2 闭环控制
```

---

# 6. 环境安装
```bash
sudo apt update
sudo apt install -y \
ros-humble-nav2* \
ros-humble-cartographer* \
ros-humble-robot-localization \
ros-humble-rviz2 \
ros-humble-teleop-twist-keyboard \
ros-humble-can-msgs \
ros-humble-socketcan-bridge \
ros-humble-dbc-parser \
python3-can python3-dbcparser
```

---

# 7. 工程创建命令
```bash
mkdir -p ~/robot_g60pro_ws/src
cd ~/robot_g60pro_ws
colcon build
source install/setup.bash
echo "source ~/robot_g60pro_ws/install/setup.bash" >> ~/.bashrc

cd src
ros2 pkg create --build-type ament_python robot_bringup
ros2 pkg create --build-type ament_python robot_base
ros2 pkg create --build-type ament_python robot_can
ros2 pkg create --build-type ament_python robot_description
ros2 pkg create --build-type ament_python robot_sensors
ros2 pkg create --build-type ament_python robot_localization
ros2 pkg create --build-type ament_python robot_slam
ros2 pkg create --build-type ament_python robot_navigation
ros2 pkg create --build-type ament_cmake robot_msgs

# 创建 robot_bringup 子文件夹
cd robot_bringup
mkdir launch config maps rviz

# 创建 robot_can DBC 目录
cd ../robot_can
mkdir config
touch config/can1.dbc config/can4.dbc
```

---

# 8. 核心启动命令
## 8.1 启动整车
```bash
ros2 launch robot_bringup robot_all.launch.py
```

## 8.2 SLAM 建图
```bash
ros2 launch robot_bringup robot_slam.launch.py
```

## 8.3 保存地图
```bash
ros2 run nav2_map_server map_saver_cli -f src/robot_bringup/maps/g60pro_map
```

## 8.4 导航
```bash
ros2 launch robot_bringup robot_navigation.launch.py
```

## 8.5 键盘控制
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

# 9. 核心 ROS 话题
- `/cmd_vel`
- `/motor_cmd`
- `/odom`
- `/imu`
- `/scan`（单线雷达 LakiBeam1s）
- `/points`（多线雷达 Helios16）
- `/camera/color` & `/camera/depth`（RGBD DCW2-DW2）
- `/can1/state`
- `/can4/motor_state`

---

# 10. 常见问题排查
- CAN 不通：检查波特率、接线、终端电阻、DBC 匹配
- 底盘不动：检查 `/cmd_vel`、CAN 收发、电机使能
- 定位漂移：检查 EKF 配置、IMU、里程计
- 导航不避障：检查雷达话题、代价地图
- 传感器无数据：检查驱动、IP、端口、权限

---

# 11. 文档版本
- V1.0：初始版本
- V1.1：完善 CAN 链路
- V1.2：加入双 DBC 支持
- V1.3：加入 robot_bringup 标准子目录
- V1.4：最终真实硬件型号版（Helios16 / LakiBeam1s / DCW2-DW2 / 安普斯 / 步科）
```

