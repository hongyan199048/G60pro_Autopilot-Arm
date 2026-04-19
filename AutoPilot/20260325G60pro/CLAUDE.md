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
./view_sensors_sim.sh      # 查看所有传感器（Gazebo）
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

`sim.launch.py` 设置四个关键环境变量：
```bash
export OGRE_RENDERER='GL'              # 软渲染避免 GPU 问题
export GAZEBO_MODEL_DATABASE_URI=''    # 禁止从网络拉取模型，避免启动卡住
export GAZEBO_MODEL_PATH=...           # 包含以下路径（按优先级）：
                                       #   robot_description share 目录（mesh 解析）
                                       #   /usr/share/gazebo-11/models（内置模型）
                                       #   ~/.gazebo/models（用户模型，含 model3）
                                       #   robot_gazebo/models（包内置模型，含 warehouse）
```

仿真世界文件：`robot_gazebo/worlds/warehouse_test.world`（仓库场景，含货架柱和箱子）

`robot_gazebo` 目录结构：
```
robot_gazebo/
├── worlds/    ← Gazebo 世界文件（.world）
│   ├── warehouse_test.world   # 当前仿真场景（sim.launch.py 加载）
│   └── parking_lot.world      # 备用停车场场景
└── models/    ← Gazebo 模型定义（model.config + model.sdf）
    └── warehouse/             # 仓库建筑模型，通过 model://warehouse 引用
```

URDF 中所有 link 的 `inertial.mass` 设为 `0.001`，车体和机械臂的 `gravity` 设为 `0`。

## 仿真 SLAM 优化记录

以下为针对仿真建图质量的历次优化，涉及文件：`robot_slam/config/cartographer_sim.lua`、`robot_gazebo/`、`robot_base/`。

### 一、TF 冲突修复

**问题**：点云整体偏高 + odom TF 高频闪烁，导致建图错位。

| 修改 | 文件 | 原因 |
|------|------|------|
| `tracking_frame` 从 `imu_link` 改为 `base_footprint` | `cartographer_sim.lua` | `imu_link` 在 odom 中 Z≠0，引起 `map→odom` Z 漂移，点云整体偏高 |
| `provide_odom_frame = false` | `cartographer_sim.lua` | 设为 true 时 Cartographer 发布 `odom→base_footprint`，与 Gazebo planar_move 冲突，造成 TF 双重发布闪烁 |
| `robot_base_node` 添加 `use_sim` 参数 | `robot_base/robot_base_node.py` | 仿真模式下 robot_base_node 不发布 odom/TF，避免与 Gazebo 的 odom 冲突 |

### 二、地图不跟随机器人运动

**问题**：Fixed Frame=map 时点云相对地图漂移；Fixed Frame=base_footprint 时地图瞬间归位。

| 修改 | 文件 | 原因 |
|------|------|------|
| `use_odometry = true` | `cartographer_sim.lua` | 关闭时 Cartographer 姿态外推器无速度信息，假设机器人静止，扫描匹配失败后 `map→odom` 不更新 |
| 更新 `motion_filter`（time=5s, distance=0.2m, angle=1°） | `cartographer_sim.lua` | 有 odom 先验后可按距离/角度触发帧插入，替代纯时间触发 |
| slam.launch.py 改用 `cartographer_sim.lua` | `robot_slam/launch/slam.launch.py` | 原 `cartographer.lua` 参数针对实车，仿真需独立配置 |

### 三、Gazebo 加载空白世界

**问题**：`sim.launch.py` 加载 `empty.world`，仿真中无任何障碍物，SLAM 无特征可匹配。

| 修改 | 文件 | 说明 |
|------|------|------|
| world 改为 `warehouse_test.world` | `sim.launch.py` | 仓库场景，含墙壁 + 货架柱 + 箱子，特征丰富 |
| GAZEBO_MODEL_PATH 新增 `~/.gazebo/models` | `sim.launch.py` | model3 车模存放于此，不加则 world 加载报错 |
| GAZEBO_MODEL_PATH 新增本地 `models/` | `sim.launch.py` | 加载 `model://warehouse` 所需 |
| `setup.py` 新增 `worlds/` 和 `models/` 安装项 | `robot_gazebo/setup.py` | colcon build 后 world/model 文件才会被安装到 share 目录 |
| 新建 `worlds/warehouse_test.world` | `robot_gazebo/worlds/` | 包含 warehouse 模型 + 6根货架柱 + 3个箱子 |
| 修复 `parking_lot.world` SDF 格式错误 | `robot_gazebo/worlds/parking_lot.world` | pillars/trash_cans/barriers 的 `<collision>` 和 `<visual>` 缺少 `name` 属性；barriers 缺少 `<collision>` 元素 |

### 四、地图墙壁双线/模糊

**问题**：建图时墙壁出现双线或粗边，沿墙方向扫描匹配"滑动"。

| 修改 | 参数 | 原值 → 新值 | 原因 |
|------|------|------------|------|
| 提高占用空间权重 | `ceres_scan_matcher.occupied_space_weight` | 1 → 10 | 增强当前帧与已有地图的对齐约束 |
| 提高平移惩罚 | `ceres_scan_matcher.translation_weight` | 10 → 100 | 压制沿墙方向的位移滑动 |
| 提高旋转惩罚 | `ceres_scan_matcher.rotation_weight` | 40 → 400 | 与 translation_weight 同比例提高，保持平衡 |

### 五、旋转时地图错位

**问题**：机器人旋转时地图发生明显偏移和错位。

| 修改 | 参数 | 说明 |
|------|------|------|
| 启用在线相关扫描匹配 | `use_online_correlative_scan_matching = true` | Ceres 是局部优化器，旋转时 odom 延迟导致初始位姿偏差，容易陷入错误极小值。RTCSM 先在搜索窗口内暴力全局搜索最优位姿再交 Ceres 精化 |
| 设置线性搜索窗口 | `linear_search_window = 0.1`（m）| 平移方向搜索范围 |
| 设置角度搜索窗口 | `angular_search_window = 20°` | 旋转方向搜索范围，覆盖 odom 的角度误差 |

---

## 注意事项

- `robot_description/launch/description.launch.py` 中 `lidar_type` 参数硬编码为 `robosense_16`，如需切换雷达需修改该文件中的 `process_file` 调用。
- 仿真 Cartographer 配置为 `cartographer_sim.lua`，实车配置为 `cartographer_real.lua`，两者**不通用**。
- 启用 `use_online_correlative_scan_matching` 后 CPU 消耗增加。若 Gazebo RTF 明显下降，可缩小 `angular_search_window`（如改为 `math.rad(10.)`）。

## Orbbec 相机

使用 `orbbec_camera/launch/dabai_dcw2.launch.py` 启动，`usb_port` 参数区分设备，需根据实际 USB 端口修改。

## RViz 配置

所有 RViz 配置文件统一在 `robot_rviz/rviz/`：

| 文件 | 用途 |
|------|------|
| `slam_sim.rviz` | Gazebo 仿真 SLAM 建图视图（Fixed Frame: map） |
| `slam_real.rviz` | 实车 SLAM 建图视图（Helios16 点云）|
| `navigation_sim.rviz` | 仿真 Nav2 导航视图（地图 + 路径 + 代价地图） |
| `sensors_view_sim.rviz` | 仿真传感器查看 |
| `sensors_view_real.rviz` | 实车传感器查看（Helios16 + 双 LakiBeam1S） |
