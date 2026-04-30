# G60Pro 实车 SLAM 系统完整流程分析

生成时间：2026-04-25

## 一、脚本总体作用

`start_slam_real.sh` 是一个**一键启动脚本**，按顺序启动 4 个核心节点，构建完整的 Cartographer SLAM 建图系统。

**核心目标**：让机器人能够：
1. 感知周围环境（激光雷达扫描）
2. 知道自己在哪里（定位）
3. 构建环境地图（建图）
4. 实时可视化（RViz）

---

## 二、启动的 4 个核心节点

### 节点 1：robot_state_publisher（TF 静态树发布）

```bash
ros2 launch robot_description description.launch.py &
```

**作用**：
- 加载机器人 URDF 模型（g60pro.urdf.xacro）
- 发布静态 TF 变换到 `/tf_static`

**发布的 TF 树**：
```
base_footprint (地面投影，Z=0)
 └─ base_link (机器人几何中心)
     ├─ rs16_link (雷达位置：前方 30cm，高 80cm)
     ├─ imu_link (IMU 位置)
     ├─ camera_front_link (前相机)
     ├─ camera_back_link (后相机)
     ├─ camera_left_link (左相机)
     ├─ camera_right_link (右相机)
     ├─ wheel_fl_link (左前轮)
     ├─ wheel_fr_link (右前轮)
     ├─ wheel_rl_link (左后轮)
     └─ wheel_rr_link (右后轮)
```

**为什么需要这个节点？**
- Cartographer 需要知道雷达相对于机器人中心的位置
- 通过 TF 查询 `base_link → rs16_link` 的变换，才能正确建图

**话题**：
- 发布：`/tf_static`（静态 TF）
- 订阅：无

---

### 节点 2：rslidar_sdk_node（激光雷达驱动）

```bash
ros2 run rslidar_sdk rslidar_sdk_node \
  --ros-args \
  -p config_path:="$WS_DIR/src/rslidar_sdk/config/config.yaml" &
```

**作用**：
- 连接 Helios16 激光雷达（IP: 192.168.2.200）
- 读取原始点云数据（UDP 端口 6699）
- 转换为 ROS2 消息格式
- 发布到 `/lidar/rs16/points`

**配置文件关键参数**（config.yaml）：
```yaml
lidar_type: RSHELIOS_16P           # 雷达型号
frame_id: rs16_link                # 点云坐标系
use_lidar_clock: false             # 使用系统时间（避免 2017 年时间戳问题）
msop_port: 6699                    # 数据端口
difop_port: 7788                   # 设备信息端口
host_address: 192.168.2.102        # 工控机 IP
lidar_address: 192.168.2.200       # 雷达 IP
```

**点云数据格式**：
```
sensor_msgs/msg/PointCloud2
  header:
    stamp: 系统时间（2026 年）
    frame_id: "rs16_link"
  fields: [x, y, z, intensity]
  width: ~28000 点/帧
  height: 1
```

**为什么需要这个节点？**
- SLAM 的输入数据源，没有点云就无法建图
- 点云的 `frame_id` 必须与 TF 树中的 `rs16_link` 一致

**话题**：
- 发布：`/lidar/rs16/points`（点云，~10Hz）
- 订阅：无

---

### 节点 3：cartographer_node（SLAM 核心）

```bash
ros2 launch robot_slam slam_real.launch.py &
```

**作用**：
- 订阅点云数据
- 执行 Scan Matching（扫描匹配）
- 构建子图（Submap）
- 执行闭环检测（Loop Closure）
- 发布地图和定位结果

**配置文件关键参数**（cartographer_real.lua）：
```lua
-- TF 配置
tracking_frame = "base_link"           -- 跟踪的坐标系（机器人中心）
published_frame = "base_footprint"     -- 发布的坐标系（地面投影）
provide_odom_frame = false             -- 不发布 odom 坐标系

-- 传感器配置
use_odometry = false                   -- 不使用轮速计（全向轮打滑严重）
use_imu_data = false                   -- 不使用 IMU（当前未启用）
num_laser_scans = 0                    -- 不使用 2D 激光
num_point_clouds = 1                   -- 使用 1 个 3D 点云

-- 扫描匹配参数
occupied_space_weight = 10             -- 占据空间权重（增强对齐）
translation_weight = 100               -- 平移权重
rotation_weight = 400                  -- 旋转权重

-- 实时闭环检测
use_online_correlative_scan_matching = true
real_time_correlative_scan_matcher = {
  linear_search_window = 0.2,          -- 平移搜索窗口 ±20cm
  angular_search_window = math.rad(20) -- 旋转搜索窗口 ±20°
}
```

**SLAM 算法流程**：

```
1. 接收点云（/lidar/rs16/points）
   ↓
2. 查询 TF（base_link → rs16_link）
   ↓
3. 将点云转换到 base_link 坐标系
   ↓
4. Scan Matching（当前帧 vs 子图）
   - 先用 Real-Time CSM 全局搜索（粗匹配）
   - 再用 Ceres 优化器精化（细匹配）
   ↓
5. 更新机器人位姿（map → base_footprint）
   ↓
6. 插入点云到子图
   ↓
7. 闭环检测（每隔一段时间）
   - 检测是否回到之前的位置
   - 如果检测到闭环，优化全局位姿图
   ↓
8. 发布结果
   - TF: map → base_footprint
   - 子图列表（供 occupancy_grid_node 使用）
```

**为什么需要这个节点？**
- SLAM 的核心算法，负责定位和建图
- 没有它就无法知道机器人在地图中的位置

**话题**：
- 订阅：`/lidar/rs16/points`（点云）
- 发布：`/tf`（动态 TF: map → base_footprint）
- 发布：`/submap_list`（子图列表，内部话题）
- 服务：`/finish_trajectory`（完成轨迹）
- 服务：`/write_state`（保存状态）

---

### 节点 4：cartographer_occupancy_grid_node（栅格地图转换）

```bash
# 由 slam_real.launch.py 同时启动
```

**作用**：
- 订阅 Cartographer 的子图列表
- 将子图转换为 2D 栅格地图（OccupancyGrid）
- 发布到 `/map`

**栅格地图格式**：
```
nav_msgs/msg/OccupancyGrid
  header:
    frame_id: "map"
  info:
    resolution: 0.05              # 5cm/格
    width: 2000                   # 地图宽度（格数）
    height: 2000                  # 地图高度（格数）
    origin:                       # 地图原点在世界坐标系中的位置
      position: {x: -50, y: -50, z: 0}
  data: [0, 0, 100, 100, -1, ...]  # 栅格值
    # 0 = 空闲（白色）
    # 100 = 占据（黑色）
    # -1 = 未知（灰色）
```

**为什么需要这个节点？**
- Cartographer 内部使用子图（Submap）格式，不是标准的 2D 地图
- Nav2 导航需要 OccupancyGrid 格式的地图
- RViz 也需要 OccupancyGrid 才能显示地图

**话题**：
- 订阅：`/submap_list`（Cartographer 内部话题）
- 发布：`/map`（栅格地图，~1Hz）

---

### 节点 5：rviz2（可视化，可选）

```bash
rviz2 -d "$WS_DIR/src/robot_rviz/rviz/slam_real.rviz" &
```

**作用**：
- 可视化点云、地图、TF 树、机器人模型
- 方便调试和监控建图过程

**显示内容**：
- RobotModel：机器人 3D 模型（基于 URDF）
- PointCloud2：雷达点云（/lidar/rs16/points）
- Map：栅格地图（/map）
- TF：坐标系轴（map, base_footprint, base_link, rs16_link）

**为什么需要这个节点？**
- 不是必须的，但强烈推荐
- 可以实时看到建图效果，发现问题

---

## 三、完整数据流图

```
┌─────────────────────────────────────────────────────────────────┐
│                        SLAM 系统数据流                           │
└─────────────────────────────────────────────────────────────────┘

硬件层：
  Helios16 雷达 (192.168.2.200)
    │ UDP 6699 端口
    ↓

节点 1：rslidar_sdk_node
    │ 发布 /lidar/rs16/points (PointCloud2, ~10Hz)
    │ frame_id: "rs16_link"
    ↓

节点 2：robot_state_publisher
    │ 发布 /tf_static
    │ TF: base_footprint → base_link → rs16_link
    ↓

节点 3：cartographer_node
    │ 订阅 /lidar/rs16/points
    │ 查询 TF: base_link → rs16_link
    │ 执行 Scan Matching
    │ 发布 /tf (map → base_footprint, ~10Hz)
    │ 发布 /submap_list (内部话题)
    ↓

节点 4：cartographer_occupancy_grid_node
    │ 订阅 /submap_list
    │ 转换为栅格地图
    │ 发布 /map (OccupancyGrid, ~1Hz)
    ↓

节点 5：rviz2
    │ 订阅 /lidar/rs16/points
    │ 订阅 /map
    │ 订阅 /tf, /tf_static
    │ 显示可视化
    ↓

用户：
    │ 键盘控制：ros2 run teleop_twist_keyboard ...
    │ 发布 /cmd_vel → (未来连接到 robot_base_node)
    │ 保存地图：./save_map.sh
```

---

## 四、TF 树的完整结构

```
运行时的完整 TF 树：

map (全局地图坐标系，固定不动)
 │
 │ ← cartographer_node 发布（动态，实时更新）
 │
 └─ base_footprint (机器人在地图中的投影，Z=0)
     │
     │ ← robot_state_publisher 发布（静态，固定）
     │
     └─ base_link (机器人几何中心)
         │
         ├─ rs16_link (雷达，前方 30cm，高 80cm)
         │   └─ 点云数据在这个坐标系中
         │
         ├─ imu_link (IMU)
         ├─ camera_front_link (前相机)
         ├─ camera_back_link (后相机)
         ├─ camera_left_link (左相机)
         ├─ camera_right_link (右相机)
         ├─ wheel_fl_link (左前轮)
         ├─ wheel_fr_link (右前轮)
         ├─ wheel_rl_link (左后轮)
         └─ wheel_rr_link (右后轮)
```

**关键点**：
- `map → base_footprint`：由 Cartographer 实时计算（机器人在地图中的位置）
- `base_footprint → base_link → rs16_link`：由 robot_state_publisher 发布（固定不变）
- 点云的 `frame_id` 是 `rs16_link`，Cartographer 通过 TF 查询将其转换到 `base_link`

---

## 五、SLAM 如何工作？

### 第 1 帧（初始化）

```
1. rslidar_sdk_node 发布第 1 帧点云
   - frame_id: "rs16_link"
   - 时间戳: t=0

2. cartographer_node 收到点云
   - 查询 TF: base_link → rs16_link（从 robot_state_publisher）
   - 将点云转换到 base_link 坐标系
   - 创建第 1 个子图（Submap 0）
   - 初始位姿：map → base_footprint = (0, 0, 0)

3. cartographer_occupancy_grid_node 收到子图
   - 转换为栅格地图
   - 发布 /map（第 1 帧地图）

4. rviz2 显示
   - 地图中心有一小块已知区域
   - 机器人在原点
```

### 第 2 帧（运动后）

```
假设机器人向前移动了 0.5m

1. rslidar_sdk_node 发布第 2 帧点云
   - frame_id: "rs16_link"
   - 时间戳: t=0.1s

2. cartographer_node 收到点云
   - 查询 TF: base_link → rs16_link
   - 将点云转换到 base_link 坐标系
   - Scan Matching：
     a. 用 Real-Time CSM 在 ±20cm, ±20° 范围内搜索最佳匹配
     b. 找到最佳位姿：(x=0.5, y=0, θ=0)
     c. 用 Ceres 优化器精化位姿：(x=0.502, y=0.001, θ=0.01)
   - 更新 TF: map → base_footprint = (0.502, 0.001, 0.01)
   - 将点云插入子图

3. cartographer_occupancy_grid_node 更新地图
   - 发布新的 /map

4. rviz2 显示
   - 地图扩大了
   - 机器人向前移动了 0.5m
```

### 第 N 帧（闭环检测）

```
假设机器人绕了一圈，回到起点附近

1. cartographer_node 收到第 N 帧点云
   - Scan Matching 发现当前位置与起点很像
   - 触发闭环检测（Loop Closure）
   - 优化全局位姿图：
     a. 调整所有历史位姿
     b. 消除累积误差
     c. 地图更加一致

2. 地图质量提升
   - 墙壁更直
   - 闭环处无重影
```

---

## 六、关键技术点

### 1. 为什么需要 TF 树？

**问题**：点云在 `rs16_link` 坐标系，但 SLAM 需要在 `base_link` 坐标系计算。

**解决**：通过 TF 查询 `base_link → rs16_link` 的变换，自动转换点云。

**如果没有 TF**：
- Cartographer 不知道雷达在哪里
- 建图会失败或错位

### 2. 为什么不使用轮速计？

**配置**：`use_odometry = false`

**原因**：
- G60Pro 是全向轮底盘，打滑严重
- 轮速计误差大，会误导 SLAM
- 纯激光 SLAM 更可靠

### 3. 为什么不使用 IMU？

**配置**：`use_imu_data = false`

**原因**：
- 当前 IMU 未校准或未连接
- 纯激光 SLAM 在室内环境足够准确
- 未来可以启用 IMU 提升旋转鲁棒性

### 4. 时间戳为什么重要？

**问题**：如果雷达时间戳是 2017 年，Cartographer 查询 TF 时会失败。

**原因**：
- TF 有时间戳，Cartographer 查询 `base_link → rs16_link` 时需要指定时间
- 如果点云时间戳是 2017 年，但 TF 是 2026 年，查询失败

**解决**：`use_lidar_clock: false`，使用系统时间。

### 5. Scan Matching 如何工作？

**目标**：找到机器人的位姿，使得当前点云与地图最匹配。

**步骤**：
1. **预测位姿**：根据上一帧位姿 + 运动模型（如果有 odom/IMU）
2. **粗匹配**：Real-Time CSM 在搜索窗口内暴力搜索
3. **细匹配**：Ceres 优化器梯度下降，最小化点云与地图的距离
4. **输出位姿**：最优的 `map → base_footprint` 变换

**为什么需要两步？**
- 粗匹配：快速找到大致位置，避免陷入局部最优
- 细匹配：精确优化，达到毫米级精度

---

## 七、如何验证 SLAM 正常工作？

### 检查 1：点云是否发布？

```bash
ros2 topic hz /lidar/rs16/points
# 应该显示 ~10Hz

ros2 topic echo /lidar/rs16/points --once
# 检查 header.stamp 是否是当前时间（2026 年）
# 检查 frame_id 是否是 "rs16_link"
```

### 检查 2：TF 树是否完整？

```bash
ros2 run tf2_tools view_frames
evince frames_*.pdf
# 应该看到：map → base_footprint → base_link → rs16_link
```

### 检查 3：Cartographer 是否运行？

```bash
ros2 node info /cartographer
# Subscriptions:
#   /lidar/rs16/points: sensor_msgs/msg/PointCloud2

ros2 topic hz /map
# 应该显示 ~1Hz
```

### 检查 4：地图是否更新？

```bash
ros2 topic echo /map --once
# 检查 data 数组是否有非 -1 的值（已知区域）
```

### 检查 5：RViz 是否正常显示？

- 点云：白色点云应该在机器人周围
- 地图：黑色墙壁，白色空闲区域
- 机器人模型：应该在地图中心
- TF 轴：红色 X，绿色 Y，蓝色 Z

---

## 八、常见问题排查

### 问题 1：RViz 中机器人飘到极远处

**原因**：雷达时间戳错误（2017 年）

**检查**：
```bash
ros2 topic echo /lidar/rs16/points --once | grep stamp
# 应该是 2026 年，不是 2017 年
```

**修复**：
```yaml
# src/rslidar_sdk/config/config.yaml
use_lidar_clock: false  # 使用系统时间
```

### 问题 2：/map 不发布

**原因**：Cartographer 崩溃或点云未收到

**检查**：
```bash
ros2 topic hz /lidar/rs16/points  # 点云是否发布？
ros2 node info /cartographer      # 节点是否运行？
```

### 问题 3：地图质量差（墙壁双线、模糊）

**原因**：Scan Matching 参数不合适

**调整**：
```lua
-- cartographer_real.lua
occupied_space_weight = 10      -- 增大（增强对齐）
translation_weight = 100        -- 增大（减少滑动）
rotation_weight = 400           -- 增大（减少旋转误差）
```

### 问题 4：建图时机器人卡顿

**原因**：CPU 负载过高（Real-Time CSM 搜索窗口太大）

**调整**：
```lua
-- cartographer_real.lua
real_time_correlative_scan_matcher = {
  angular_search_window = math.rad(10)  -- 从 20° 减小到 10°
}
```

---

## 九、总结

### SLAM 系统的 4 个核心组件

| 组件 | 节点 | 作用 |
|------|------|------|
| **传感器驱动** | rslidar_sdk_node | 提供点云数据 |
| **机器人模型** | robot_state_publisher | 提供 TF 静态树 |
| **SLAM 算法** | cartographer_node | 定位 + 建图 |
| **地图转换** | cartographer_occupancy_grid_node | 子图 → 栅格地图 |

### 数据流总结

```
雷达硬件 → rslidar_sdk_node → /lidar/rs16/points
                                      ↓
robot_state_publisher → /tf_static → cartographer_node
                                      ↓
                              /tf (map → base_footprint)
                              /submap_list
                                      ↓
                        cartographer_occupancy_grid_node
                                      ↓
                                   /map
                                      ↓
                                   rviz2
```

### 关键技术

1. **TF 变换**：连接不同坐标系，让点云能正确转换
2. **Scan Matching**：通过匹配点云与地图，计算机器人位姿
3. **子图**：分块建图，提高效率
4. **闭环检测**：消除累积误差，提高地图一致性

### 为什么这样设计？

- **模块化**：每个节点职责单一，易于调试和替换
- **标准化**：使用 ROS2 标准消息和 TF，兼容其他工具
- **可扩展**：未来可以加入 IMU、轮速计、多雷达融合
