# robot_description

**机器人模型与 TF 树**

## 功能

定义 G60Pro 机器人的 URDF/XACRO 模型，发布静态 TF 树和机器人描述。

## 核心文件

### `urdf/g60pro.urdf.xacro`

**机器人配置**：
- **底盘**：4 轮 8 驱全向移动平台
  - 车体尺寸：1.5m × 1.0m × 1.0m
  - 离地间隙：0.155m
  - 轮子半径：0.15m
  - 前后轮距：1.22m（半距 0.61m）
  - 左右轮距：0.541m（半距 0.2705m）

- **传感器**：
  - 1 × Helios16 多线激光雷达（或 Livox Mid360）
  - 2 × LakiBeam1S 单线激光雷达
  - 4 × Orbbec DaBai DCW2 RGBD 相机
  - 1 × IMU

**参数化配置**：
- `lidar_type` — 切换多线雷达类型：
  - `robosense_16`（默认）— RoboSense Helios16
  - `livox_mid360` — Livox Mid360

---

## TF 树结构

```
base_footprint (地面投影点)
    ↓
base_link (车体中心)
    ├─ rs16_link (Helios16，高度 1.54m)
    ├─ imu_link (IMU)
    ├─ lidar_single_1_link (右前单线雷达，高度 0.4m)
    ├─ lidar_single_2_link (左后单线雷达，高度 0.4m)
    ├─ camera_front_link (前相机)
    ├─ camera_back_link (后相机)
    ├─ camera_left_link (左相机)
    ├─ camera_right_link (右相机)
    ├─ wheel_fl_link (左前轮)
    ├─ wheel_fr_link (右前轮)
    ├─ wheel_rl_link (左后轮)
    └─ wheel_rr_link (右后轮)
```

**坐标系定义**：
- `base_footprint` — 地面投影点（Z=0）
- `base_link` — 车体中心（Z=0.155m，离地间隙）
- 所有传感器坐标系相对 `base_link` 定义

---

## 启动文件

### `launch/description.launch.py`

**功能**：
- 加载 URDF 模型
- 启动 `robot_state_publisher` 节点
- 发布 `/robot_description` 话题
- 发布静态 TF 树

**启动命令**：
```bash
ros2 launch robot_description description.launch.py
```

**参数**：
- `use_sim_time` — 是否使用仿真时间（默认 false）
- `lidar_type` — 雷达类型（硬编码为 `robosense_16`）

---

## 传感器位置

| 传感器 | 位置（相对 base_link） | 说明 |
|--------|----------------------|------|
| Helios16 | (0, 0, 1.39m) | 离地 1.54m |
| 单线雷达 1 | (0.6, 0.3, 0.25m) | 右前，离地 0.4m |
| 单线雷达 2 | (-0.6, -0.3, 0.25m) | 左后，离地 0.4m |
| 前相机 | (0.75, 0, 0.5m) | 车头中央 |
| 后相机 | (-0.75, 0, 0.5m) | 车尾中央 |
| 左相机 | (0, 0.5, 0.5m) | 车身左侧 |
| 右相机 | (0, -0.5, 0.5m) | 车身右侧 |

---

## 仿真配置

### Gazebo 插件

**`gazebo_ros_planar_move`**（仿真专用）：
- 发布 TF：`odom → base_footprint`
- 发布 `/odom` 话题（地面真值）
- 订阅 `/cmd_vel`，驱动机器人运动

**`gazebo_ros_ray_sensor`**（仿真专用）：
- 模拟 16 线激光雷达
- 发布 `/lidar/multi/points`

**质量与重力**：
- 所有 link `mass = 0.001`（轻量化，避免物理引擎计算负担）
- 车体和机械臂 `gravity = 0`（避免下沉）

---

## 切换雷达类型

### 方法 1：修改 launch 文件（推荐）

编辑 `launch/description.launch.py`：
```python
lidar_type = DeclareLaunchArgument(
    'lidar_type',
    default_value='livox_mid360',  # 改为 livox_mid360
    description='激光雷达类型'
)
```

### 方法 2：启动时传参

```bash
ros2 launch robot_description description.launch.py \
  lidar_type:=livox_mid360
```

---

## 关键话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/robot_description` | String | URDF 模型（XML 格式） |
| `/tf_static` | TFMessage | 静态 TF 树 |

---

## 依赖

- `robot_state_publisher` — 发布 TF 树
- `xacro` — URDF 宏处理
- `orbbec_description` — Orbbec 相机 URDF 模型

---

## 注意事项

1. **轮距参数一致性**：URDF 中的轮距（0.61 / 0.2705）必须与 `robot_base` 节点中的参数一致
2. **雷达高度**：Helios16 离地 1.54m，单线雷达离地 0.4m，需与实车测量值一致
3. **仿真与实车差异**：
   - 仿真：`gazebo_ros_planar_move` 发布 `odom → base_footprint`
   - 实车：`robot_base_node` 发布 `odom → base_footprint`（可选）
4. **TF 冲突**：实车纯定位模式下，`robot_base_node` 不应发布 `odom → base_footprint`，避免与 Cartographer 冲突
