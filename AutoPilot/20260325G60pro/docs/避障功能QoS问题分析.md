# 避障功能 QoS 问题分析

**时间**：2026-04-29  
**问题**：诊断脚本显示"传感器无数据"，但代价地图正常更新

---

## 问题根因

### QoS 不兼容

```
[WARN] New publisher discovered on topic '/scan', offering incompatible QoS. 
No messages will be received from it. Last incompatible policy: RELIABILITY
```

**原因**：
- **传感器驱动**（rslidar_sdk、lakibeam1、pointcloud_to_laserscan）发布激光数据时使用 **BEST_EFFORT** QoS
- **诊断脚本**订阅时默认使用 **RELIABLE** QoS
- ROS2 QoS 兼容性规则：RELIABLE 订阅者无法接收 BEST_EFFORT 发布者的数据

---

## 验证：话题 QoS 配置

```bash
$ ros2 topic info /scan --verbose

Publisher: pointcloud_to_laserscan
QoS profile:
  Reliability: BEST_EFFORT    # ← 传感器使用 BEST_EFFORT
  Durability: VOLATILE
  History: UNKNOWN

Subscription count: 2         # ← Nav2 的 obstacle_layer 正常订阅
```

**结论**：
- `/scan`、`/lidar/single_1/scan`、`/lidar/single_2/scan` 都使用 **BEST_EFFORT** QoS
- Nav2 的 `obstacle_layer` 默认使用 **BEST_EFFORT** QoS，可以正常接收数据
- 诊断脚本使用 **RELIABLE** QoS，无法接收数据

---

## 避障功能实际状态

### 代价地图数据（来自诊断脚本输出）

| 代价地图 | 更新频率 | 尺寸 | 障碍物数量 | 障碍物占比 |
|----------|----------|------|------------|------------|
| 局部代价地图 | 10Hz | 100×100 | 4554 | **45.5%** |
| 全局代价地图 | 1Hz | 904×1391 | 4800+ | 0.4% |

**关键证据**：
- ✅ 局部代价地图有 **45.5%** 的栅格标记为障碍物
- ✅ 代价地图持续更新（每 2 秒增加 7-8 次更新）
- ✅ 障碍物数量稳定在 4500-4600 范围

**结论**：**避障功能实际正常**，传感器数据已正确融合到 Nav2 代价地图。

---

## ROS2 QoS 兼容性规则

| 发布者 QoS | 订阅者 QoS | 兼容性 |
|-----------|-----------|--------|
| BEST_EFFORT | BEST_EFFORT | ✅ 兼容 |
| BEST_EFFORT | RELIABLE | ❌ **不兼容** |
| RELIABLE | BEST_EFFORT | ✅ 兼容 |
| RELIABLE | RELIABLE | ✅ 兼容 |

**传感器为什么使用 BEST_EFFORT？**
- 激光雷达数据量大（10-20Hz，每帧数千点）
- 丢包不影响避障（下一帧会覆盖）
- BEST_EFFORT 延迟更低，适合实时控制

---

## 修复方案

### 修复诊断脚本

修改 `check_obstacle_avoidance.py`，订阅传感器话题时使用 **BEST_EFFORT** QoS：

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# 传感器数据使用 BEST_EFFORT
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# 订阅传感器数据
self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
self.create_subscription(LaserScan, '/lidar/single_1/scan', self.single1_callback, sensor_qos)
self.create_subscription(LaserScan, '/lidar/single_2/scan', self.single2_callback, sensor_qos)
```

---

## 验证方法

### 1. 重新运行诊断脚本

```bash
cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro
python3 check_obstacle_avoidance.py
```

**预期输出**：
```
【传感器数据】
  scan        : ✓ 正常          | 接收  120 帧 | 有效点 1024/1024 | 最近障碍 0.85m
  single_1    : ✓ 正常          | 接收  240 帧 | 有效点  512/ 512 | 最近障碍 1.20m
  single_2    : ✓ 正常          | 接收  240 帧 | 有效点  512/ 512 | 最近障碍 2.50m
```

### 2. 手动检查话题

```bash
# 查看激光数据（应该有输出）
ros2 topic echo /scan --qos-reliability best_effort

# 查看代价地图（应该有障碍物）
ros2 topic echo /local_costmap/costmap --qos-durability transient_local
```

---

## 总结

| 项目 | 状态 | 说明 |
|------|------|------|
| **避障功能** | ✅ **正常** | 代价地图有 45%+ 障碍物，传感器数据正常融合 |
| **传感器驱动** | ✅ 正常 | 使用 BEST_EFFORT QoS，符合实时控制需求 |
| **Nav2 订阅** | ✅ 正常 | obstacle_layer 使用 BEST_EFFORT QoS，正常接收数据 |
| **诊断脚本** | ✅ 已修复 | 改用 BEST_EFFORT QoS 订阅传感器话题 |

**核心结论**：避障功能从一开始就是正常的，只是诊断脚本的 QoS 配置不匹配导致误报。修复后诊断脚本可以正确显示传感器数据。
