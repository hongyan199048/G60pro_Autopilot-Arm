# robot_slam

**定位层 — Cartographer SLAM + 纯定位**

## 功能

基于 Google Cartographer 的 2D SLAM 系统，提供两种工作模式：

1. **建图模式**（Mapping）：实时构建地图 + 定位
2. **纯定位模式**（Localization Only）：加载已保存地图，只做定位

## 工作模式

### 模式 1：SLAM 建图

**配置文件**：
- 仿真：`config/cartographer_sim.lua`
- 实车：`config/cartographer_real.lua`

**启动文件**：
- 仿真：`launch/slam_sim.launch.py`
- 实车：`launch/slam_real.launch.py`

**功能**：
- 订阅激光点云，实时构建 2D 栅格地图
- 发布 `/map` 话题（OccupancyGrid）
- 发布 TF：`map → base_footprint`（实车）或 `map → odom`（仿真）

**使用场景**：第一次在新环境中建图

**启动命令**：
```bash
# 实车建图
ros2 launch robot_slam slam_real.launch.py

# 保存地图
ros2 run nav2_map_server map_saver_cli -f maps/my_map
ros2 service call /cartographer/write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: 'maps/my_map.pbstream'}"
```

---

### 模式 2：纯定位

**配置文件**：`config/cartographer_real_localization.lua`

**启动文件**：`launch/slam_real_localization.launch.py`

**功能**：
- 加载 `.pbstream` 地图文件（Cartographer 内部格式）
- 只做 scan matching 定位，不修改地图
- 不发布 `/map` 话题（避免与 map_server 冲突）
- 发布 TF：`map → base_footprint`

**使用场景**：在已建好的地图中导航

**启动命令**：
```bash
ros2 launch robot_slam slam_real_localization.launch.py \
  pbstream_file:=/path/to/map.pbstream
```

---

## 配置文件对比

| 参数 | 建图模式（实车） | 纯定位模式 |
|------|----------------|-----------|
| `tracking_frame` | `base_link` | `base_link` |
| `published_frame` | `base_footprint` | `base_footprint` |
| `use_odometry` | `false` | `false` |
| `use_imu_data` | `false` | `false` |
| Z 高度过滤 | -0.6m ~ 1.4m | -0.6m ~ 1.4m |
| 距离过滤 | 1.6m ~ 30.0m | 1.6m ~ 30.0m |
| RTCSM 搜索窗口 | 角度 20° | 角度 30°，线性 0.5m |

---

## 关键话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/lidar/rs16/points` | PointCloud2 | 输入：Helios16 点云 |
| `/map` | OccupancyGrid | 输出：栅格地图（仅建图模式） |
| TF: `map → base_footprint` | — | 输出：定位结果 |

---

## 注意事项

1. **时间同步**：`rslidar_sdk` 必须设置 `use_lidar_clock: false`，使用系统时间
2. **初始位姿**：纯定位模式下，必须在 RViz 中手动设置初始位姿（2D Pose Estimate）
3. **地图格式**：
   - `.pbstream` — Cartographer 内部格式，用于纯定位
   - `.pgm/.yaml` — 栅格地图，用于 Nav2 全局代价地图

---

## 依赖

- `cartographer_ros` — Google Cartographer ROS2 包
- `rslidar_sdk` — Helios16 驱动
- `robot_description` — URDF 模型（提供 TF 树）
