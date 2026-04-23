# robot_navigation

**规划层 — Nav2 导航栈**

## 功能

基于 ROS2 Nav2 的自主导航系统，提供全局路径规划、局部避障、行为控制等功能。

## 核心组件

### 1. 全局路径规划（Global Planner）

**算法**：NavFn Planner（A* 算法）

**功能**：
- 在静态地图上规划从起点到终点的最优路径
- 考虑地图障碍物和代价地图

### 2. 局部路径规划（Local Planner）

**算法**：DWB Local Planner（Dynamic Window Approach）

**功能**：
- 实时避障，动态调整路径
- 考虑机器人运动学约束（最大速度、加速度）
- 评估多条候选轨迹，选择最优轨迹

**关键参数**（实车优化）：
- `vtheta_samples: 20` — 降低角速度采样，减少摆动
- `sim_time: 2.5s` — 增大预测时间，轨迹更平滑
- `PathAlign.scale: 16.0` — 降低路径对齐权重
- `PathDist.scale: 48.0` — 提高路径距离权重

### 3. 行为控制（Behavior Server）

**功能**：
- 旋转恢复（Spin Recovery）
- 后退恢复（Backup Recovery）
- 等待行为（Wait）

### 4. 速度平滑（Velocity Smoother）

**功能**：
- 平滑速度指令，避免急加速/急减速
- 输入：`/cmd_vel_nav`
- 输出：`/cmd_vel`

---

## 工作模式

### 仿真模式（`navigation_sim.launch.py`）

**定位方案**：AMCL（粒子滤波）

**启动命令**：
```bash
ros2 launch robot_navigation navigation_sim.launch.py \
  map:=/path/to/map.yaml
```

### 实车模式（`navigation_real.launch.py`）

**定位方案**：Cartographer 纯定位（不使用 AMCL）

**启动命令**：
```bash
ros2 launch robot_navigation navigation_real.launch.py \
  map:=/path/to/map.yaml \
  pbstream_file:=/path/to/map.pbstream
```

**三种地图模式**（互斥）：
1. **纯定位模式**：有 `.pbstream` → Cartographer 从文件加载地图
2. **静态地图模式**：有 `.yaml` → map_server 发布静态地图
3. **实时建图模式**：无 `.yaml` 无 `.pbstream` → cartographer_occupancy_grid_node 发布实时地图

---

## 关键节点

| 节点 | 功能 |
|------|------|
| `planner_server` | 全局路径规划 |
| `controller_server` | 局部路径跟踪（DWB） |
| `behavior_server` | 行为恢复 |
| `bt_navigator` | 行为树导航逻辑 |
| `smoother_server` | 路径平滑 |
| `velocity_smoother` | 速度平滑 |
| `waypoint_follower` | 多航点跟随 |
| `map_server` | 静态地图发布（可选） |
| `pointcloud_to_laserscan` | 3D 点云转 2D 扫描 |
| `initial_pose_relay` | 处理 RViz 初始位姿设置 |

---

## 配置文件

| 文件 | 说明 |
|------|------|
| `config/nav2_params_sim.yaml` | 仿真参数（含 AMCL） |
| `config/nav2_params_real.yaml` | 实车参数（无 AMCL，Cartographer 定位） |

---

## 关键话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/goal_pose` | PoseStamped | 输入：导航目标点 |
| `/cmd_vel` | Twist | 输出：速度指令 |
| `/map` | OccupancyGrid | 输入：静态地图 |
| `/scan` | LaserScan | 输入：2D 激光扫描（代价地图） |
| `/local_costmap/costmap` | OccupancyGrid | 输出：局部代价地图 |
| `/global_costmap/costmap` | OccupancyGrid | 输出：全局代价地图 |

---

## 使用流程

### 1. 启动导航栈

```bash
# 实车导航
ros2 launch robot_navigation navigation_real.launch.py \
  map:=maps/g60pro_v6.yaml \
  pbstream_file:=maps/g60pro_v6.pbstream
```

### 2. 设置初始位姿（RViz）

点击 RViz 工具栏的 "2D Pose Estimate"，在地图上点击机器人当前位置和朝向。

### 3. 发送导航目标（RViz）

点击 RViz 工具栏的 "2D Nav Goal"，在地图上点击目标位置和朝向。

### 4. 监控导航状态

```bash
# 查看导航状态
ros2 topic echo /navigate_to_pose/_action/status

# 查看速度指令
ros2 topic echo /cmd_vel
```

---

## 代价地图配置

### 全局代价地图（Global Costmap）

- **坐标系**：`map`
- **更新频率**：1 Hz
- **障碍物层**：静态地图 + 激光扫描
- **膨胀层**：膨胀半径 0.55m

### 局部代价地图（Local Costmap）

- **坐标系**：`map`（实车）/ `odom`（仿真）
- **更新频率**：5 Hz
- **尺寸**：5m × 5m（机器人周围）
- **障碍物层**：激光扫描 + 点云
- **膨胀层**：膨胀半径 0.55m

---

## 注意事项

1. **实车必须设置初始位姿**：Cartographer 纯定位模式下，必须手动设置初始位姿
2. **AMCL 仅用于仿真**：实车使用 Cartographer 定位，不启动 AMCL
3. **点云高度过滤**：`pointcloud_to_laserscan` 高度范围 0.1m ~ 2.0m（相对 base_footprint）
4. **速度限制**：实车最大线速度 0.3 m/s，角速度 1.0 rad/s（保守值，可调整）

---

## 依赖

- `nav2_bringup` — Nav2 核心包
- `robot_slam` — Cartographer 定位
- `robot_description` — URDF 模型
- `pointcloud_to_laserscan` — 点云转换
