# 单线雷达 Costmap 标记问题分析

## 问题描述
单线雷达数据正常，Nav2 配置正确，但 costmap 中无法生成障碍物膨胀区。

## 已验证的事实

### ✅ 数据层面
- 单线雷达发布有效数据：197 个点在 2.5m 以内
- 话题：`/lidar/single_1/scan` 正常发布
- frame_id：`single_lidar_1_link`
- 数据格式：`sensor_msgs/msg/LaserScan`

### ✅ 配置层面
```yaml
observation_sources: scan single_1 single_2
single_1:
  topic: /lidar/single_1/scan
  sensor_frame: single_lidar_1_link
  marking: true
  clearing: true
  min_obstacle_height: -2.0
  max_obstacle_height: 3.0
  obstacle_min_range: 0.0
  obstacle_max_range: 2.5
  data_type: LaserScan
```

### ✅ 订阅层面
- local_costmap 确实订阅了 `/lidar/single_1/scan`
- 有 3 个订阅者（rviz + local_costmap + global_costmap）

### ✅ TF 层面
- `map → odom → base_footprint → base_link → single_lidar_1_link` 链存在
- TF 正常发布（从 `/tf` 话题确认）
- transform_tolerance: 1.0 秒

## 可能的原因

### 1. **高度过滤问题**（已排除）
- 单线雷达离地 0.486m
- `min_obstacle_height: -2.0` 足够低
- 低矮障碍物应该能通过

### 2. **距离过滤问题**（部分可能）
- `obstacle_max_range: 2.5m`
- 如果障碍物都在 2.5m 以外，会被过滤
- **需要在 RViz 中确认障碍物实际距离**

### 3. **TF 查询失败**（可能性高）
- 虽然 TF 在发布，但 Nav2 查询时可能失败
- 时间戳不同步？
- TF 缓冲区延迟？

### 4. **数据更新频率问题**（可能）
- `expected_update_rate: 10.0 Hz`
- 实际频率可能不足，导致数据被标记为过期

### 5. **clearing 与 marking 冲突**（可能）
- `clearing: true` 可能在清除 `marking: true` 标记的障碍物
- 需要测试：临时禁用 clearing，只保留 marking

## 下一步排查

### 方案 1：RViz 可视化验证
```bash
# 在 RViz 中添加：
# 1. /lidar/single_1/scan (LaserScan)
# 2. /local_costmap/costmap (Map)
# 3. 观察红色点（障碍物）是否在 2.5m 以内
# 4. 观察 costmap 中对应位置是否有标记
```

### 方案 2：临时禁用 clearing
```bash
ros2 param set /local_costmap/local_costmap obstacle_layer.single_1.clearing false
# 等待 5 秒
# 观察 costmap 是否出现障碍物
```

### 方案 3：增大 obstacle_max_range
```bash
ros2 param set /local_costmap/local_costmap obstacle_layer.single_1.obstacle_max_range 5.0
# 如果障碍物在 2.5-5m 之间，应该能看到
```

### 方案 4：启用 Nav2 调试日志
```bash
ros2 param set /local_costmap/local_costmap use_sim_time false
ros2 run rqt_console rqt_console
# 过滤 /local_costmap 节点的日志
# 查找 "observation" 或 "marking" 相关的警告/错误
```

## 参考信息

- 单线雷达驱动：lakibeam1_scan_node
- 扫描范围：45° ~ 315°（270° 扇区）
- 右前雷达朝向：车头右前 -45°（URDF yaw）
- 左后雷达朝向：车尾左后 135°（URDF yaw）
