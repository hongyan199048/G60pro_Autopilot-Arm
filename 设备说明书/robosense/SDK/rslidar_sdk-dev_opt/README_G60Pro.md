# rslidar_sdk 与 G60 Pro AutoPilot 对接说明

本目录为 **RoboSense rslidar_sdk**（含 **rs_driver** 驱动内核），支持 **Helios 16**（RSHELIOS_16P）等型号，用于**实车**时接入多线雷达、发布 PointCloud2，供 Fast-LIO2 建图/定位与 HAL 使用。

---

## 1. 在工程里的作用

| 用途 | 说明 |
|------|------|
| **实车多线雷达驱动** | 连接 Helios 16 硬件，接收 UDP 点云/差分包（msop_port 6699、difop_port 7788），解码后发布 ROS2 PointCloud2。 |
| **与 AutoPilot 对接** | 将点云话题配置为 `/lidar/rs16/points`、frame 为 `rs16_link`，即可与 `g60pro_launch/config/hal/rs16.yaml`、Fast-LIO2（`lidar_type: 2` VELO16）、vehicle_hal 等一致。 |
| **点云格式** | 支持 **XYZIRT**（x, y, z, intensity, **ring**, timestamp），与 Fast-LIO 的 velodyne_handler 所需格式一致，建图/定位可直接用。 |

仿真时不需要本 SDK（Gazebo 已发布 `/lidar/rs16/points`）；**实车**时必须用本 SDK（或同协议驱动）才能拿到真实雷达数据。

---

## 2. 支持的雷达型号（节选）

- **RSHELIOS_16P** → **Helios 16**（16 线，与本项目当前配置一致）
- RS-LiDAR-16、RS32、RS-Helios、RS-Bpearl、RS-Ruby、RS-LiDAR-M1 等

Helios 16 对应配置中 `lidar_type: RSHELIOS_16P`。

---

## 3. 工程结构简要

```
rslidar_sdk-dev_opt/
├── src/
│   ├── rs_driver/                    # 驱动内核（解码、通信）
│   │   └── driver/decoder/
│   │       ├── decoder_RSHELIOS_16P.hpp   # Helios 16 解码
│   │       ├── decoder_RS16.hpp
│   │       └── ...
│   └── source/                       # 数据源与 ROS 封装
│       ├── source_driver.hpp         # 在线雷达 → 点云
│       └── source_pointcloud_ros.hpp # 点云 → ROS/ROS2 PointCloud2（含 ring）
├── node/rslidar_sdk_node.cpp         # ROS/ROS2 节点入口
├── config/config.yaml                # 雷达类型、端口、话题、frame
├── launch/start.py                   # ROS2 launch 示例
├── package_ros1.xml / package_ros2.xml
└── CMakeLists.txt                    # 可选 CATKIN 或 COLCON 编译
```

---

## 4. 与 G60 Pro 对齐的配置要点

在 **config/config.yaml** 中建议（或新建一份 `config_helios16_g60pro.yaml`）：

```yaml
common:
  msg_source: 1                       # 1: 在线雷达
  send_point_cloud_ros: true
lidar:
  - driver:
      lidar_type: RSHELIOS_16P       # Helios 16
      msop_port: 6699
      difop_port: 7788
      host_address: "0.0.0.0"
      # start_angle / end_angle / min_distance / max_distance 按需
    ros:
      ros_frame_id: rs16_link         # 与 URDF、Fast-LIO、hal/rs16.yaml 一致
      ros_send_point_cloud_topic: /lidar/rs16/points
```

编译时建议将 **POINT_TYPE** 设为 **XYZIRT**（CMakeLists.txt 中 `set(POINT_TYPE XYZIRT)`），以带 **ring** 和 **timestamp**，便于 Fast-LIO 使用。

---

## 5. 编译与运行（ROS2）

- 当前 CMake 默认 **COMPILE_METHOD: CATKIN**；用于 ROS2 时需改为 **COLCON**，并满足 ROS2、**rslidar_msg**、yaml-cpp 等依赖（见 README_CN.md）。
- 将本包放入 ROS2 工作空间 `src/`，使用 `package_ros2.xml` 作为 `package.xml`（或重命名/切换），然后：

  ```bash
  colcon build --packages-select rslidar_sdk
  source install/setup.bash
  ros2 run rslidar_sdk rslidar_sdk_node --ros-args -p config_path:=/path/to/config_helios16_g60pro.yaml
  ```

- 若 launch 里写死 `config_path`，可同样指向上述 yaml，并保证 **ros_send_point_cloud_topic** 与 **ros_frame_id** 与 G60 Pro 一致。

---

## 6. 小结

| 项目 | 说明 |
|------|------|
| **Helios 16 支持** | 有，使用 `decoder_RSHELIOS_16P.hpp`，config 中 `lidar_type: RSHELIOS_16P`。 |
| **输出** | PointCloud2，可带 ring/timestamp（XYZIRT），与 Fast-LIO VELO16 及本车 `rs16_link`、`/lidar/rs16/points` 对接。 |
| **在 G60 Pro 中的位置** | 实车 HAL 层：接 Helios 16 硬件 → 发布 `/lidar/rs16/points` → 供 vehicle_localization（Fast-LIO2）、vehicle_perception、上层规划控制使用。 |
| **仿真** | 不需要本 SDK；仿真用 Gazebo 的 rs16 插件即可。 |

更多编译、依赖、参数说明见 **README_CN.md** / **README.md** 及 **doc/** 下说明。
