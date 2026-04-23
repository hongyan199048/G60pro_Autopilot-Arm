# orbbec_camera_msgs

**Orbbec 相机消息定义**

## 功能

定义 Orbbec 相机专用的 ROS2 消息类型。

## 消息列表

### 1. `Extrinsics.msg` — 外参矩阵

**用途**：相机外参标定

**定义**：
```
float64[9] rotation     # 旋转矩阵（3×3）
float64[3] translation  # 平移向量（3×1）
```

### 2. `Metadata.msg` — 元数据

**用途**：图像元数据（时间戳、曝光、增益等）

**定义**：
```
std_msgs/Header header
string json
```

### 3. `DeviceInfo.msg` — 设备信息

**用途**：相机设备信息

**定义**：
```
string name
string serial_number
string firmware_version
string hardware_version
```

---

## 使用说明

**注意**：当前项目主要使用标准 ROS2 消息（`sensor_msgs/Image`、`sensor_msgs/PointCloud2`），不直接使用 `orbbec_camera_msgs`。

`orbbec_camera_msgs` 主要用于：
- 相机标定
- 设备管理
- 高级功能（IMU、多相机同步等）

---

## 编译

```bash
colcon build --packages-select orbbec_camera_msgs
```

---

## 依赖

- `std_msgs` — ROS2 标准消息
- `rosidl_default_generators` — 消息生成器

---

## 注意事项

1. **与 orbbec_camera 配套**：此包由 orbbec_camera 依赖，不单独使用
2. **标准消息优先**：应用层优先使用 `sensor_msgs`，除非需要 Orbbec 特有功能
3. **版本兼容**：确保与 orbbec_camera 版本匹配
