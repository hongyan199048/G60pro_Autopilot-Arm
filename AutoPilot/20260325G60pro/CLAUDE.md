# 重要：每次回复必须以"Tim_20260325G60pro，"开头，无一例外。

# CLAUDE.md

工作空间：`/home/admin123/Development/G60Pro/AutoPilot/20260325G60pro`

## 重要约束

**强制不使用轮式里程计**：本项目定位方案完全依赖纯激光雷达 Cartographer SLAM（Helios16 多线雷达），不使用底盘轮速计 `/odom`。配置文件中 `use_odometry = false`。原因：全向轮打滑严重，轮速计误差大。

**IMU 暂不使用**：当前 `use_imu_data = false`，后续可考虑融合 IMU 提升旋转鲁棒性。

**定位精度刚需**：到达目的地时，位置容差 ≤5cm，角度偏差 ±7.5°。这是充电对接的硬性要求，所有导航参数调整必须满足此约束。

## 编译

```bash
cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build
```

## 启动

命名规则：`_sim` = Gazebo 仿真，`_real` = 实车

### 仿真
```bash
cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro/docs
./start_slam_sim.sh        # Gazebo + Cartographer SLAM + RViz
./start_nav_sim.sh         # 建图 → 保存地图 → Nav2 导航
./view_sensors_sim.sh      # 查看所有传感器
```

### 实车
```bash
cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro/docs
./start_slam_real.sh       # Helios16 + Cartographer + RViz
./view_sensors_real.sh     # 查看所有传感器（Helios16 + 双 LakiBeam1S）
./start_nav_real.sh --yaml g60pro_v6        # Nav2 导航 + RViz
./save_map.sh              # 保存当前 /map 为 maps/g60pro_v*.pgm
```

### 键盘控制
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/cmd_vel
```

### 监控
```bash
cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro/docs
./monitor_nav.sh
```

## 系统架构

| 层级 | 包 | 职责 |
|------|-----|------|
| 感知 | `robot_slam` / `rslidar_sdk` / `orbbec_camera` | 激光雷达 + RGBD；Cartographer 2D SLAM |
| 定位 | `robot_localization` | EKF 融合 odom + IMU → `/odom_combined` |
| 规划 | `robot_navigation` | Nav2 全局/局部规划 |
| 控制 | `robot_base` / `robot_can` | 底盘运动学解算 + 双 CAN 总线通信 |
| 模型 | `robot_description` | URDF/XACRO 机器人模型 + TF |

**数据流：**
```
/cmd_vel → robot_base_node → /motor_cmd (CAN) + /odom + TF(odom→base_footprint)
/lidar/multi/points → Cartographer → /map + /scan
/lidar/single_1/2/scan → Nav2 避障
/imu → Cartographer + EKF
```

**关键话题：** `/cmd_vel`、`/odom`、`/motor_cmd`、`/motor_state`、`/lidar/multi/points`、`/lidar/single_1/2/scan`、`/imu`

**TF 树：** `odom → base_footprint → base_link → (rs16_link, camera_*_link, imu_link, wheel_*, ...)`

- 仿真：`odom → base_footprint` 由 `gazebo_ros_planar_move` 发布
- 实车：由 `robot_base_node` 发布

**启动入口：**

| 文件 | 用途 |
|------|------|
| `robot_gazebo/launch/sim.launch.py` | Gazebo 仿真 |
| `robot_slam/launch/slam_real.launch.py` | 实车 Cartographer（`cartographer_real.lua`）|
| `robot_slam/launch/slam.launch.py` | 仿真 Cartographer（`cartographer_sim.lua`）|
| `robot_navigation/launch/navigation.launch.py` | Nav2 导航 |
| `robot_base/launch/base.launch.py` | 底盘节点 |
| `robot_description/launch/description.launch.py` | URDF + TF |

## 机器人模型

`robot_description/urdf/g60pro.urdf.xacro`

- 支持 `lidar_type` 参数切换雷达：`robosense_16`（默认）/ `livox_mid360`
- 注意：`description.launch.py` 中 `lidar_type` 硬编码为 `robosense_16`，需切换时修改该文件
- 4 轮 8 驱 + 4 路 Orbbec RGBD 相机 + 2 路单线雷达 + IMU

## CAN 通信

**CAN 总线配置**：
- CAN1（can0，500kbps）：工控机 ↔ RDM 控制器双向通信
- DBC 协议文件：`robot_can/config/can1/G60_CAN1_RDM_V1.0.dbc`

**关键 CAN ID（基于 DBC V1.0）**：

| 方向 | CAN ID | 十进制 | 消息名称 | 说明 | 频率 |
|------|--------|--------|----------|------|------|
| Tx | 0x210 | 528 | LAS_Fr01 | 四轮驱动转速指令 (RPM) | 20Hz |
| Tx | 0x211 | 529 | LAS_Fr02 | 四轮转向角指令 (deg) | 20Hz |
| Tx | 0x212 | 530 | LAS_Fr03 | 任务状态帧 | 20Hz |
| Rx | 0x1FF | 511 | RDM_Fr04 | 整车状态反馈（驾驶模式、任务状态、充电状态） | - |
| Rx | 0x19 | 25 | RDM_Fr35 | 四轮转速反馈 (RPM) | - |
| Rx | 0x1B | 27 | RDM_Fr36 | 四轮转向角反馈 (deg) | - |
| Rx | 0x2A | 42 | RDM_Fr37 | FL/FR 电机位置 (32bit signed) | - |
| Rx | 0x2B | 43 | RDM_Fr38 | RL/RR 电机位置 (32bit signed) | - |

**ROS2 话题映射**：
- `/motor_cmd` (MotorCmd) → CAN 0x210 + 0x211
- CAN 0x19 + 0x1B → `/motor_state` (MotorState)
- CAN 0x1FF → `/veh_status` (CanFrame)
- 原始 CAN 帧 → `/can_frames` (CanFrame, 调试用)

## Gazebo 仿真

`sim.launch.py` 设置环境变量：
```bash
export OGRE_RENDERER='GL'           # 软渲染
export GAZEBO_MODEL_DATABASE_URI='' # 禁止从网络拉模型
export GAZEBO_MODEL_PATH=...        # robot_description share + 内置 + ~/.gazebo/models + robot_gazebo/models
```

世界文件：`robot_gazebo/worlds/warehouse_test.world`（含 warehouse 模型 + 货架柱 + 箱子）

URDF：所有 link `mass=0.001`，车体和机械臂 `gravity=0`

## 仿真 SLAM 优化（cartographer_sim.lua）

| # | 问题 | 修改 | 原因 |
|---|------|------|------|
| 1 | TF 闪烁 + 点云偏高 | `tracking_frame: base_footprint`，`provide_odom_frame: false` | `imu_link` 在 odom 中 Z≠0 引起漂移；与 Gazebo planar_move 冲突 |
| 2 | 地图不跟随机器人 | `use_odometry: true`，`motion_filter: time=5s/dist=0.2m/angle=1°` | 无 odom 时 Cartographer 假设机器人静止，scan matching 失败 |
| 3 | 加载空白世界 | world 改为 `warehouse_test.world` + 修复 `parking_lot.world` SDF | `empty.world` 无特征，SLAM 无法匹配 |
| 4 | 墙壁双线/模糊 | `occupied_space_weight: 10`，`translation_weight: 100`，`rotation_weight: 400` | 增强当前帧与地图的对齐约束，压制滑动 |
| 5 | 旋转时地图错位 | `use_online_correlative_scan_matching: true`，`angular_search_window: 20°` | Ceres 局部优化，旋转时陷入错误极小值；RTCSM 先全局搜索再精化 |

注意：仿真用 `cartographer_sim.lua`，实车用 `cartographer_real.lua`，两者不通用。启用 RTCSM 后若 CPU 压力大，可缩小 `angular_search_window` 至 `math.rad(10.)`。

## 实车 SLAM 排查

### 2026-04-20：map 坐标系跳变（百万米量级）

**症状**：Fixed Frame=`map` 时机器人飘到极远处，`map→base_footprint` 每次跳变百万米。

**排查**：TF 静态树正常 → 问题在 Cartographer 发布的这段 TF → 用 Python 读点云 stamp：`1483237008` = **2017年1月19日**（与系统时间差 9 年）

**根因**：`rslidar_sdk/config/config.yaml` 中 `use_lidar_clock: true`，雷达内部 RTC 从未校准，停留在出厂默认值（2017 年）。时间戳错误导致 Cartographer TF 查找失败，scan matching 收到垃圾数据。

**修复**：`use_lidar_clock: false`（使用工控机系统时间），重启 rslidar_sdk_node 或重新运行 `./start_slam_real.sh`。

**长期方案**：工控机搭建 NTP Server（Chrony），Helios16 通过 NTP 对时到工控机，再改回 `use_lidar_clock: true`。

## 其他

**Orbbec 相机**：`orbbec_camera/launch/dabai_dcw2.launch.py`，`usb_port` 参数按实际修改。

**RViz 配置**：`robot_rviz/rviz/` 下各 `.rviz` 文件用途：
- `slam_sim.rviz` / `slam_real.rviz` — 建图视图
- `navigation_sim.rviz` — Nav2 导航（含代价地图）
- `sensors_view_sim.rviz` / `sensors_view_real.rviz` — 传感器查看

**时间同步**：所有传感器（雷达、IMU 等）必须与工控机时间同步，建议工控机为 NTP Server，避免时间戳跳变问题。