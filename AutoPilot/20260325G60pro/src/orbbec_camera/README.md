# orbbec_camera

**Orbbec RGBD 相机驱动**

## 功能

Orbbec 官方 ROS2 驱动，支持 DaBai DCW2 等 RGBD 深度相机。

## 支持型号

- **DaBai DCW2**（当前使用）
- Astra
- Gemini
- Femto
- 等

---

## 启动文件

### `launch/dabai_dcw2.launch.py`

**功能**：启动单个 DaBai DCW2 相机

**启动命令**：
```bash
ros2 launch orbbec_camera dabai_dcw2.launch.py
```

**参数**：
- `usb_port` — USB 端口号（需根据实际修改）
- `camera_name` — 相机名称（默认 `camera`）
- `depth_registration` — 深度对齐到彩色（默认 `true`）

---

## 关键话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/color/image_raw` | Image | 彩色图像（RGB） |
| `/camera/depth/image_raw` | Image | 深度图像（16bit） |
| `/camera/depth/points` | PointCloud2 | 深度点云 |
| `/camera/color/camera_info` | CameraInfo | 彩色相机内参 |
| `/camera/depth/camera_info` | CameraInfo | 深度相机内参 |

---

## 多相机配置

G60Pro 配置 4 路 Orbbec 相机：

| 相机 | 位置 | USB 端口 | 话题前缀 |
|------|------|---------|---------|
| 前相机 | 车头中央 | `/dev/video0` | `/camera_front/` |
| 后相机 | 车尾中央 | `/dev/video2` | `/camera_back/` |
| 左相机 | 车身左侧 | `/dev/video4` | `/camera_left/` |
| 右相机 | 车身右侧 | `/dev/video6` | `/camera_right/` |

**注意**：USB 端口号需根据实际连接情况修改。

---

## 用途

### 1. 障碍物检测

**方案**：
- 深度图像 → 障碍物检测算法
- 点云 → Nav2 局部代价地图

### 2. 视觉定位（未来）

**方案**：
- RGB 图像 → 视觉 SLAM（ORB-SLAM3）
- 与 Cartographer 融合，提升定位精度

### 3. 目标识别（未来）

**方案**：
- RGB 图像 → YOLO 目标检测
- 识别充电口、货架、箱子等

---

## 故障排查

### 1. 相机无法打开

**排查步骤**：

1. **检查 USB 连接**：
   ```bash
   lsusb | grep Orbbec
   ```

2. **检查设备权限**：
   ```bash
   ls -l /dev/video*
   sudo chmod 666 /dev/video*
   ```

3. **检查驱动加载**：
   ```bash
   dmesg | grep video
   ```

### 2. 图像数据异常

**症状**：图像全黑或全白

**排查**：
- 检查相机镜头是否被遮挡
- 检查曝光参数设置
- 检查 USB 带宽（USB 3.0 优先）

### 3. 深度图像无效

**症状**：深度图像全为 0 或 inf

**排查**：
- 检查红外发射器是否工作
- 检查测量距离（有效范围 0.3m ~ 5m）
- 检查环境光照（强光下深度失效）

---

## 依赖

- `libuvc` — USB 视频类驱动
- `sensor_msgs` — ROS2 图像消息
- `image_transport` — 图像传输
- `cv_bridge` — OpenCV 桥接

---

## 注意事项

1. **USB 带宽**：4 路相机同时运行需要 USB 3.0 带宽，建议分配到不同 USB 控制器
2. **设备权限**：需要 root 权限或将用户加入 `video` 组
3. **深度对齐**：`depth_registration: true` 会增加 CPU 负载
4. **帧率限制**：深度 + 彩色同时输出，建议帧率 ≤ 30 FPS
5. **当前状态**：驱动已集成，但多相机启动脚本待完善

---

## 待完成工作

- [ ] 编写 4 路相机统一启动脚本
- [ ] 集成到 Nav2 局部代价地图
- [ ] 优化 USB 带宽分配
- [ ] 相机内外参标定
- [ ] 深度图像滤波与优化

---

## 官方文档

- GitHub: https://github.com/orbbec/OrbbecSDK_ROS2
- 用户手册: https://github.com/orbbec/OrbbecSDK_ROS2/blob/main/README.md
