# robot_rviz

**RViz 配置文件**

## 功能

提供 G60Pro 项目的 RViz 可视化配置文件，用于不同场景的数据可视化。

## 配置文件列表

### 1. `slam_sim.rviz` — 仿真建图视图

**用途**：Gazebo 仿真 + Cartographer SLAM 建图

**显示内容**：
- 机器人模型（RobotModel）
- TF 树（TF）
- 激光点云（PointCloud2）：`/lidar/multi/points`
- 地图（Map）：`/map`
- 里程计（Odometry）：`/odom`

**Fixed Frame**：`map`

---

### 2. `slam_real.rviz` — 实车建图视图

**用途**：实车 Helios16 + Cartographer SLAM 建图

**显示内容**：
- 机器人模型
- TF 树
- 激光点云：`/lidar/rs16/points`
- 地图：`/map`

**Fixed Frame**：`map`

---

### 3. `navigation_sim.rviz` — 仿真导航视图

**用途**：Gazebo 仿真 + Nav2 导航

**显示内容**：
- 机器人模型
- TF 树
- 激光点云：`/lidar/multi/points`
- 地图：`/map`
- 全局代价地图：`/global_costmap/costmap`
- 局部代价地图：`/local_costmap/costmap`
- 全局路径：`/plan`
- 局部路径：`/local_plan`
- 目标点：`/goal_pose`

**交互工具**：
- 2D Pose Estimate — 设置初始位姿
- 2D Nav Goal — 发送导航目标

**Fixed Frame**：`map`

---

### 4. `navigation_real.rviz` — 实车导航视图

**用途**：实车 Helios16 + Nav2 导航

**显示内容**：
- 机器人模型
- TF 树
- 激光点云：`/lidar/rs16/points`
- 2D 激光扫描：`/scan`
- 地图：`/map`
- 全局代价地图：`/global_costmap/costmap`
- 局部代价地图：`/local_costmap/costmap`
- 全局路径：`/plan`
- 局部路径：`/local_plan`

**交互工具**：
- 2D Pose Estimate — 设置初始位姿（**必须！**）
- 2D Nav Goal — 发送导航目标

**Fixed Frame**：`map`

---

### 5. `sensors_view_sim.rviz` — 仿真传感器查看

**用途**：查看所有传感器数据（仿真）

**显示内容**：
- 机器人模型
- TF 树
- 激光点云：`/lidar/multi/points`
- RGBD 相机（4 路）：
  - `/camera_front/color/image_raw`
  - `/camera_back/color/image_raw`
  - `/camera_left/color/image_raw`
  - `/camera_right/color/image_raw`

**Fixed Frame**：`base_link`

---

### 6. `sensors_view_real.rviz` — 实车传感器查看

**用途**：查看所有传感器数据（实车）

**显示内容**：
- 机器人模型
- TF 树
- Helios16 点云：`/lidar/rs16/points`
- 单线雷达 1：`/lidar/single_1/scan`
- 单线雷达 2：`/lidar/single_2/scan`
- RGBD 相机（4 路）

**Fixed Frame**：`base_link`

---

## 使用方法

### 启动 RViz

```bash
# 仿真建图
rviz2 -d src/robot_rviz/rviz/slam_sim.rviz

# 实车建图
rviz2 -d src/robot_rviz/rviz/slam_real.rviz

# 实车导航
rviz2 -d src/robot_rviz/rviz/navigation_real.rviz

# 传感器查看
rviz2 -d src/robot_rviz/rviz/sensors_view_real.rviz
```

---

## 交互工具使用

### 2D Pose Estimate（设置初始位姿）

1. 点击 RViz 工具栏的 "2D Pose Estimate" 按钮
2. 在地图上点击机器人当前位置
3. 拖动鼠标设置朝向
4. 释放鼠标

**用途**：
- 实车纯定位模式下，**必须**手动设置初始位姿
- 仿真 AMCL 定位时，可选（粒子滤波会自动收敛）

### 2D Nav Goal（发送导航目标）

1. 点击 RViz 工具栏的 "2D Nav Goal" 按钮
2. 在地图上点击目标位置
3. 拖动鼠标设置目标朝向
4. 释放鼠标

**用途**：
- 发送导航目标点到 Nav2
- 机器人会自动规划路径并导航到目标点

---

## 常见问题

### 1. TF 树显示红色

**原因**：TF 变换缺失或超时

**排查**：
```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 检查 TF 变换
ros2 run tf2_ros tf2_echo map base_footprint
```

### 2. 点云不显示

**原因**：话题名称错误或数据未发布

**排查**：
```bash
# 查看话题列表
ros2 topic list

# 查看点云数据
ros2 topic echo /lidar/rs16/points --no-arr
```

### 3. 地图不显示

**原因**：`/map` 话题未发布

**排查**：
```bash
# 查看地图话题
ros2 topic info /map

# 检查 Cartographer 或 map_server 是否启动
ros2 node list
```

### 4. 代价地图显示异常

**原因**：Nav2 节点未启动或配置错误

**排查**：
```bash
# 查看 Nav2 节点状态
ros2 lifecycle list

# 查看代价地图话题
ros2 topic echo /local_costmap/costmap --no-arr
```

---

## 依赖

- `rviz2` — ROS2 可视化工具
- `rviz_common` — RViz 核心库
- `rviz_default_plugins` — RViz 默认插件

---

## 注意事项

1. **Fixed Frame 选择**：
   - 建图/导航：`map`（全局坐标系）
   - 传感器查看：`base_link`（机器人本体坐标系）

2. **实车初始位姿**：
   - Cartographer 纯定位模式下，**必须**手动设置初始位姿
   - 否则定位偏差大，导航失败

3. **配置文件保存**：
   - 修改 RViz 配置后，记得保存（File → Save Config As）
   - 保存到 `src/robot_rviz/rviz/` 目录
