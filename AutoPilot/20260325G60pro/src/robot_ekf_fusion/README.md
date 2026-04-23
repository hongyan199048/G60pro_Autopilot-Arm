# robot_ekf_fusion

**状态：备用方案，当前未启用**

## 功能

使用官方 `robot_localization` 包的 EKF 节点，融合 IMU + 里程计数据，提供更精确的定位。

## 当前项目定位方案

当前项目使用 **Cartographer 纯定位模式**（`robot_slam` 包），不需要 EKF 融合。

- **建图模式**：`cartographer_real.lua` — 实时 SLAM 建图 + 定位
- **纯定位模式**：`cartographer_real_localization.lua` — 加载 `.pbstream` 地图，只做定位

## 何时使用 EKF 融合

如果未来遇到以下场景，可以考虑启用 EKF 融合：

1. **特征稀疏区域**：长走廊、空旷场地，Cartographer scan matching 不稳定
2. **高动态运动**：快速转弯、加速，需要 IMU 提供高频姿态估计
3. **多传感器融合**：需要融合 GPS、视觉里程计等额外传感器

## 使用方法

```bash
# 启动 EKF 融合节点
ros2 launch robot_ekf_fusion ekf.launch.py

# 输出话题：/odometry/filtered
# TF：odom → base_link
```

## 配置文件

- `config/ekf.yaml` — EKF 参数配置
- `launch/ekf.launch.py` — 启动文件

## 注意事项

- EKF 输出的 TF（`odom → base_link`）会与 Cartographer 的 TF（`map → base_footprint`）共存
- 需要确保 TF 树不冲突：`map → odom → base_link → base_footprint`
