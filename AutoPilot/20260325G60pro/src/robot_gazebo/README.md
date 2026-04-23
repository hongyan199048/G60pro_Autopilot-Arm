# robot_gazebo

**Gazebo 仿真环境**

## 功能

提供 G60Pro 机器人的 Gazebo 仿真环境，包括世界文件、模型、仿真配置。

## 核心文件

### `launch/sim.launch.py`

**功能**：
- 启动 Gazebo 服务器和客户端
- 加载世界文件
- Spawn 机器人模型到仿真环境
- 设置环境变量

**启动命令**：
```bash
ros2 launch robot_gazebo sim.launch.py
```

---

## 世界文件

### `worlds/warehouse_test.world`

**场景**：仓库环境

**内容**：
- Warehouse 建筑模型
- 货架柱（用于 SLAM 特征）
- 箱子（障碍物）
- 地面平面

**用途**：SLAM 建图测试、导航测试

---

## 环境变量配置

`sim.launch.py` 自动设置以下环境变量：

```bash
export OGRE_RENDERER='GL'           # 软渲染（避免 GPU 兼容性问题）
export GAZEBO_MODEL_DATABASE_URI='' # 禁止从网络拉模型（加速启动）
export GAZEBO_MODEL_PATH=...        # 模型搜索路径
```

**模型搜索路径**（优先级从高到低）：
1. `robot_description/share/robot_description/models`
2. Gazebo 内置模型路径
3. `~/.gazebo/models`
4. `robot_gazebo/models`

---

## 模型目录

### `models/`

**包含模型**：
- `warehouse/` — 仓库建筑
- `pallet/` — 货架柱
- `box/` — 箱子

**注意**：模型已从 `worlds/warehouse/` 移动到 `models/warehouse/`，避免路径混淆。

---

## Gazebo 插件

### 1. `gazebo_ros_planar_move`

**功能**：
- 订阅 `/cmd_vel`，驱动机器人运动
- 发布 `/odom` 话题（地面真值）
- 发布 TF：`odom → base_footprint`

**参数**：
- `odometry_frame`: `odom`
- `robot_base_frame`: `base_footprint`
- `update_rate`: 50 Hz

### 2. `gazebo_ros_ray_sensor`

**功能**：
- 模拟 16 线激光雷达
- 发布 `/lidar/multi/points` 点云

**参数**：
- 垂直分辨率：16 线
- 水平分辨率：0.2°
- 扫描范围：360°
- 距离范围：0.5m ~ 30m

---

## URDF 仿真优化

### 质量与重力

**问题**：默认质量过大，导致物理引擎计算慢、机器人下沉

**优化**：
- 所有 link `mass = 0.001`（轻量化）
- 车体和机械臂 `gravity = 0`（避免下沉）

### 碰撞模型

**简化碰撞几何**：
- 车体：长方体
- 轮子：圆柱体
- 传感器：无碰撞（`collision` 标签省略）

---

## 启动流程

### 完整仿真启动

```bash
# 1. 启动 Gazebo 仿真
ros2 launch robot_gazebo sim.launch.py

# 2. 启动 SLAM 建图
ros2 launch robot_slam slam_sim.launch.py

# 3. 启动 RViz
rviz2 -d src/robot_rviz/rviz/slam_sim.rviz

# 4. 键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/cmd_vel
```

---

## 常见问题

### 1. Gazebo 启动慢

**原因**：尝试从网络下载模型

**解决**：
- 已设置 `GAZEBO_MODEL_DATABASE_URI=''`，禁止网络下载
- 确保所有模型在本地 `models/` 目录

### 2. 渲染黑屏

**原因**：GPU 驱动不兼容

**解决**：
- 已设置 `OGRE_RENDERER='GL'`，使用软渲染
- 如果仍有问题，尝试 `export LIBGL_ALWAYS_SOFTWARE=1`

### 3. 机器人下沉

**原因**：质量过大或重力设置错误

**解决**：
- 已设置 `mass=0.001` 和 `gravity=0`
- 检查 URDF 中所有 link 的 `<inertial>` 标签

### 4. 点云数据异常

**原因**：激光雷达插件配置错误

**解决**：
- 检查 `gazebo_ros_ray_sensor` 插件配置
- 确认话题名称 `/lidar/multi/points`
- 检查 `frame_name` 是否为 `rs16_link`

---

## 依赖

- `gazebo_ros` — Gazebo ROS2 接口
- `robot_description` — 机器人 URDF 模型
- `gazebo_ros_pkgs` — Gazebo 插件

---

## 注意事项

1. **仿真与实车差异**：
   - 仿真：`gazebo_ros_planar_move` 提供完美里程计
   - 实车：`robot_base_node` 提供开环里程计（误差大）

2. **Cartographer 配置**：
   - 仿真：`cartographer_sim.lua`（`use_odometry: true`）
   - 实车：`cartographer_real.lua`（`use_odometry: false`）

3. **编译产物清理**：
   - `src/robot_gazebo/build|install|log` 是误生成的编译产物，已删除
   - 正确的编译产物在顶层 `build/install/log/` 目录
