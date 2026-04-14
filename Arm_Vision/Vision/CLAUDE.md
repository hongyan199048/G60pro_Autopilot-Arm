# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

G60Pro 自动充电机器人视觉模块。使用 YOLO11 + Orbbec DaBai DCW2 深度相机识别直流充电口，通过两级位姿估算（粗位姿 + ICP 精定位）引导 Flexiv Rizon10 机械臂完成充电枪插接。

## 两级位姿估算流程

```
机械臂搜索位 → [粗位姿估算] → 机械臂移动到标准拍摄位 → [ICP 精确定位] → 机械臂插接
                      ↑                                        ↑
              YOLO + 深度 + RANSAC                        模板点云 + ICP
```

**粗位姿**（`pose_estimation.py` / `vision_node.py`）：
- YOLO11 检测 `DC_charging_port` 边框
- bbox 区域深度中值反投影 → 充电座中心 3D 坐标
- bbox 区域点云 RANSAC 平面拟合 → 法向量（Z轴）
- 法向量 + 世界Y轴参考 → 正交化旋转矩阵
- 精度：平移 ~10mm，旋转 ~5°

**精位姿**（`icp_refine.py` / `icp_node.py`）：
- YOLO 检测确定充电口区域
- 深度图 → 当前帧点云（target）
- CAD STEP → 模板点云（source，中心在原点）
- 以标准拍摄位（Z=500mm）为初始值，运行 PointToPlane ICP
- 精度：平移 ±0.5mm，旋转 ±1°

## 运行环境

- **Python**：`/home/admin123/miniconda3/envs/yolo_env`
- **ROS2**：Humble
- **相机 SDK**：`pyorbbecsdk/`（需编译，`.so` 在 `pyorbbecsdk/install/lib/`）
- **依赖**：ultralytics（YOLO11）、opencv-python、numpy、open3d、pythonocc-core（STEP读取）、rclpy

## 启动方式

### 纯 Python（调试用，无需 ROS2）

```bash
bash run_pose_estimation.sh   # 粗位姿（OpenCV 可视化）
bash run_icp_refine.sh       # ICP 精定位（OpenCV 可视化）
```

### ROS2 节点（集成到机械臂系统）

```bash
# 构建
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select arm_vision
source install/setup.bash

# 运行
ros2 launch arm_vision arm_vision.launch.py
```

**发布话题**：
- `/arm/coarse_pose` — `geometry_msgs/PoseStamped`（粗位姿）
- `/arm/coarse_debug` — `std_msgs/Float64MultiArray` [x,y,z,rx,ry,rz,conf]
- `/arm/fine_pose` — `geometry_msgs/PoseStamped`（精位姿）
- `/arm/fine_debug` — `std_msgs/Float64MultiArray` [x,y,z,rx,ry,rz,fitness,inlier_rmse(mm)]

## 关键算法文件

| 文件 | 作用 |
|------|------|
| `arm_vision/arm_vision/utils.py` | 核心算法工具（frame_to_bgr, backproject, fit_plane_ransac, build_coarse_pose 等） |
| `arm_vision/arm_vision/vision_node.py` | ROS2 粗位姿节点 |
| `arm_vision/arm_vision/icp_node.py` | ROS2 ICP 精定位节点 |
| `step_to_pcd.py` | CAD STEP → 模板点云（一次性，运行后输出 `.pcd/.ply`） |

## 模板点云生成

当 CAD 文件（`datasets/直流充电座STP和点云/国标直流充电插座.STEP`）更新时：

```bash
/home/admin123/miniconda3/envs/yolo_env/bin/python step_to_pcd.py
```

输出到 `datasets/直流充电座STP和点云/charging_port_template.pcd`。

## YOLO 模型重新训练

```bash
conda activate yolo_env
yolo detect train \
  model=models/yolo11n.pt \
  data="datasets/DC charging port.v2i.yolov11/data.yaml" \
  epochs=100 \
  imgsz=512 \
  project=detect \
  name=dc_charging_v3
```

正式版权重：`detect/dc_charging_v2i/weights/best.pt`

## ROS2 包结构

```
arm_vision/                    # 独立于 ros2_ws 的 ROS2 源码包
├── arm_vision/
│   ├── vision_node.py         # 粗位姿节点（相机线程 + ROS2 定时发布）
│   ├── icp_node.py            # 精位姿节点
│   └── utils.py               # 算法工具（不依赖 ROS2/OpenCV）
├── launch/
│   └── arm_vision.launch.py   # 同时启动两个节点，设置 LD_LIBRARY_PATH
└── ros2_ws/src/
    └── arm_vision → ../arm_vision   # 符号链接

ros2_ws/                       # colcon build 输出目录
```

节点通过 `setup.py` 的 `entry_points` 注册为 console_scripts：`vision_node` 和 `icp_node`。

## 注意事项

- `pyorbbecsdk/` 重新编译后，确保 `LD_LIBRARY_PATH` 包含 `pyorbbecsdk/install/lib`
- `T_cam2gripper`（手眼标定矩阵）当前硬编码为单位矩阵，需替换为实际标定结果
- `datasets/直流充电座STP和点云/` 存放 CAD 和模板点云，`datasets/` 下其他目录为 YOLO 数据集
- `OrbbecViewer_*` 工具仅用于相机参数调试，不参与运行时

## 未完成项

- 手眼标定（相机与机械臂末端的相对位置关系）
- 机械臂运动控制（Flexiv SDK 集成，引导机械臂从搜索位 → 标准拍摄位）
- 力控插枪策略（力控模式 + 插接成功检测）
