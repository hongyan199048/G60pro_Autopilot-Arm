# rslidar_msg

**RoboSense 激光雷达消息定义**

## 功能

定义 RoboSense 激光雷达专用的 ROS2 消息类型。

## 消息列表

### 1. `RslidarScan.msg` — 原始扫描数据

**用途**：雷达原始数据包

**定义**：
```
std_msgs/Header header
rslidar_msg/RslidarPacket[] packets
```

### 2. `RslidarPacket.msg` — 单个数据包

**用途**：雷达单帧数据

**定义**：
```
std_msgs/Header header
uint8[] data
```

---

## 使用说明

**注意**：当前项目使用 `sensor_msgs/PointCloud2` 作为点云数据格式，不直接使用 `rslidar_msg`。

`rslidar_msg` 主要用于：
- 雷达驱动内部数据传输
- 原始数据录制与回放
- 低级别雷达调试

---

## 编译

```bash
colcon build --packages-select rslidar_msg
```

---

## 依赖

- `std_msgs` — ROS2 标准消息
- `rosidl_default_generators` — 消息生成器

---

## 注意事项

1. **与 rslidar_sdk 配套**：此包由 rslidar_sdk 依赖，不单独使用
2. **点云格式**：应用层使用 `sensor_msgs/PointCloud2`，不直接使用此消息
3. **版本兼容**：确保与 rslidar_sdk 版本匹配
