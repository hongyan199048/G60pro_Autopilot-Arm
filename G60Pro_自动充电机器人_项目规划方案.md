# G60Pro 自动充电机器人 — 项目规划方案（精简版）

> 基于详细需求调研后的修订方案。核心原则：**1个月，1人，极度聚焦核心功能**。

---

## 一、项目约束与范围

### 1.1 关键约束
| 约束项 | 内容 |
|--------|------|
| **工期** | 1个月内完成 |
| **人员** | 1人开发 |
| **验收标准** | 能完成基本充电流程即可 |
| **目标车型** | 多车型通用 |
| **充电枪** | GB/T 20234.3 标准枪（任意型号） |

### 1.2 已有基础（直接可用）
| 模块 | 状态 | 来源 |
|------|------|------|
| YOLO11 充电口检测 | **已验证可用** | `Arm_Vision/pose_estimation.py` |
| ICP 精定位 | 模型已验证 | `Arm_Vision/icp_refine.py` |
| Orbbec RGBD 相机 | **ROS2驱动已集成** | `orbbec_sdk_ros2` |
| AutoPilot ROS2 工作空间 | 可编译，基础功能骨架 | `AutoPilot/20260325G60pro/` |
| G60Pro 自动驾驶底盘 | **完整可用** | 硬件就绪 |
| 云端通信 | **同事完成中** | 不需要我做 |
| BMS/GB27930 充电协议 | **同事完成中** | 不需要我做 |

### 1.3 本月核心任务（只做这些）
```
机械臂SDK集成 → 手眼标定 → 视觉ROS2节点 → 运动规划 → 力控插枪 → 急停安全
```

---

## 二、系统架构（精简版）

### 2.1 架构原则
- AutoPilot 和 Arm **无需强联动**：各自独立运行，通过少量状态信号协调
- 云端/BMS 由同事负责，本月方案**不涉及**
- 视觉引导作为独立节点，输出位姿给机械臂

### 2.2 模块关系
```
┌──────────────┐     CAN/状态信号      ┌──────────────┐
│  AutoPilot   │ ◄──────────────────► │     Arm      │
│  (导航/停车)  │   0x300/0x301        │ (Flexiv SDK) │
└──────────────┘                      └──────┬───────┘
       │                                     │
       │ ROS2话题                            │ ROS2话题
       │ /camera/...                         │ /arm/coarse_pose
       ▼                                     ▼
┌──────────────┐                      ┌──────────────┐
│  Orbbec相机   │ ──────────────────► │ 视觉引导节点  │
│  (ROS2驱动)   │   /camera/color/...  │ (YOLO+ICP)   │
└──────────────┘                      └──────────────┘
```

### 2.3 ROS2 话题（精简）
```
/camera/color/image_raw    # 相机图像（Orbbec驱动已有）
/camera/depth/image_raw     # 深度图像（Orbbec驱动已有）
/arm/coarse_pose           # 粗位姿（YOLO+RANSAC）
/arm/fine_pose             # 精位姿（ICP）
/arm/command               # 机械臂指令（MoveJ/MoveL/Stop）
/arm/state                 # 机械臂状态反馈
/arm/emergency_stop        # 急停信号
```

---

## 三、实施计划（4周，精确到天）

### 第1周：机械臂基础 + 视觉ROS2节点

**Day 1-2：Flexiv SDK 环境搭建**
- 安装 Flexiv Rizon10 SDK（Python/C++）
- 测试机械臂基本运动（关节空间 MoveJ）
- 编写 `arm_driver` ROS2 节点
- 验证：机械臂可通过 `/arm/command` 话题运动

**Day 3-4：视觉引导 ROS2 节点**
- 将 `pose_estimation.py` 和 `icp_refine.py` 整合为 `arm_vision_node`
- 订阅 `/camera/color/image_raw` 和 `/camera/depth/image_raw`
- 发布 `/arm/coarse_pose` 和 `/arm/fine_pose`
- 验证：视觉节点持续输出位姿，与原脚本行为一致

**Day 5：手眼标定（简化版）**
- 使用棋盘格标定法求相机内参（如果尚未标定）
- 用机械臂末端触碰标定板特征点，建立 T_cam2gripper
- 简化验证：移动机械臂末端到标定板，误差 < 1cm

### 第2周：运动规划 + 力控插枪

**Day 6-8：机械臂运动规划**
- 设计运动路径：home位 → 搜索位 → 接近位 → 插枪位
- 实现 `arm_motion_planner` 节点
- 订阅 `/arm/fine_pose`，计算机械臂目标位姿
- 调用 Flexiv SDK 执行 MoveL
- 验证：机械臂可从 home 移动到目标点

**Day 9-10：力控插枪策略**
- Flexiv SDK 力控模式使能
- 实现力控插入逻辑：
  1. 粗对准（视觉引导）→ 末端接近充电口
  2. 切换力控模式（限力 10-15N）
  3. 沿 Z 轴缓慢插入
  4. 检测到位信号（深度到位 or 力突增）
- 验证：在测试架上完成插枪动作

### 第3周：AutoPilot 对接 + 急停安全 + 仿真

**Day 11-12：AutoPilot 协同对接**
- 实现 CAN 0x300（AutoPilot→Arm）和 0x301（Arm→AutoPilot）收发
- 简化流程：
  - AutoPilot 导航到目标 → 发布 0x300=对接就绪
  - Arm 收到 → 开始视觉检测 + 插枪
  - Arm 完成 → 发布 0x301=对接完成
  - AutoPilot 收到 → 可执行下一步
- 修改 `vehicle_decision/state_machine.py` 添加充电相关状态

**Day 13-14：急停安全机制**
- 实现急停按钮监听（GPIO 或 ROS2 topic）
- 急停触发：立即停止机械臂运动
- 急停触发：向 AutoPilot 发布急停信号
- 验证：按下急停，机械臂立即停止

**Day 15：Gazebo 仿真环境搭建**
- 创建充电站简化模型（停车位 + 充电口几何体）
- 创建客户EV简化模型（车身 + 充电口位置）
- 机械臂 URDF 集成到 Gazebo（使用 Flexiv 官方 URDF 或简化模型）
- 验证：可在 Gazebo 中看到充电站 + EV + 机械臂

### 第4周：联调 + 测试 + 迭代

**Day 16-18：全流程联调（仿真）**
- 场景1：导航到充电站 → 视觉检测 → 机械臂插枪（仿真）
- 场景2：模拟充电完成 → 拔枪 → 机械臂归位（仿真）
- 修复发现的问题

**Day 19-20：全流程联调（实车）**
- 场景1：机械臂单独运动测试
- 场景2：视觉引导 + 机械臂对准测试
- 场景3：完整插枪流程测试
- 场景4：急停安全性测试

**Day 21-22：参数调优与鲁棒性**
- 视觉检测阈值调优
- 力控参数调优（插入速度、限力值）
- 运动规划路径优化
- 多车型适配测试

**Day 23-25：文档与验收准备**
- 整理代码，编写 README
- 记录关键参数（T_cam2gripper、力控阈值、运动路径）
- 准备验收演示

**Day 26-28（缓冲）：问题修复与迭代**
- 处理联调中发现的边界情况
- 插枪失败重试逻辑
- 异常处理与日志记录

---

## 四、ROS2 包结构

```
AutoPilot/20260325G60pro/src/
├── arm_control/                    # [新建] 机械臂控制包
│   ├── arm_control/
│   │   ├── __init__.py
│   │   ├── arm_driver.py           # Flexiv SDK 桥接，关节/MoveJ/MoveL/力控
│   │   ├── force_controller.py    # 力控插枪逻辑
│   │   └── emergency_stop.py      # 急停按钮监听
│   ├── launch/
│   │   └── arm_control.launch.py
│   └── package.xml
│
├── arm_vision/                     # [新建] 视觉引导包
│   ├── arm_vision/
│   │   ├── __init__.py
│   │   ├── vision_node.py         # 整合 pose_estimation + icp_refine
│   │   └── hand_eye_calibration.py # 手眼标定工具
│   ├── launch/
│   │   └── arm_vision.launch.py
│   └── package.xml
│
├── arm_navigation/                 # [新建] 机械臂运动规划包
│   ├── arm_navigation/
│   │   ├── __init__.py
│   │   └── motion_planner.py       # 路径规划：home→搜索位→插枪位
│   ├── launch/
│   │   └── arm_navigation.launch.py
│   └── package.xml
│
├── vehicle_decision/               # [修改] 扩展状态机
│   └── vehicle_decision/
│       └── state_machine.py        # 添加 CHARGING 相关状态
│
├── vehicle_hal/                     # [修改] CAN 协同
│   └── vehicle_hal/
│       └── chassis_can_driver.py   # 添加 0x300/0x301 收发
│
└── g60pro_gazebo/                   # [修改] 仿真环境
    └── g60pro_gazebo/
        └── models/
            ├── charging_station/    # [新建] 充电站模型
            └── ev_simplified/       # [新建] 简化EV模型
```

---

## 五、消息定义

### 5.1 新增消息

```msg
# ArmCommand.msg
string motion_type   # "movej" / "movel" / "stop" / "enable_force" / "disable_force"
float64[6] target_pose      # 笛卡尔目标位姿 [x,y,z,rx,ry,rz] 单位m/rad
float64[6] target_joints    # 关节目标角度
float64 max_velocity
float64 max_acceleration
float64 max_force           # 力控最大力（N），仅 force 模式
```

```msg
# ArmState.msg
builtin_interfaces/Time stamp
uint8 IDLE=0
uint8 MOVING=1
uint8 FORCE_MODE=2
uint8 PLUGGING=3
uint8 PLUGGED=4
uint8 UNPLUGGING=5
uint8 FAULT=9
uint8 state
float64[6] end_effector_pose
float64[6] current_joints
float64[6] force_torque
bool force_control_active
```

### 5.2 CAN 协同信号（简化）

| CAN ID | 方向 | 内容 |
|--------|------|------|
| 0x300 | AutoPilot → Arm | 车辆状态：0=运动中, 1=已停稳, 2=对接就绪, 3=离开许可 |
| 0x301 | Arm → AutoPilot | 机械臂状态：0=空闲, 1=对接中, 2=对接完成, 3=拔枪完成, 9=故障 |

---

## 六、关键文件修改清单

| 文件 | 操作 | 修改内容 |
|------|------|---------|
| `arm_control/arm_control/arm_driver.py` | 新建 | Flexiv SDK 桥接 |
| `arm_control/arm_control/force_controller.py` | 新建 | 力控插枪逻辑 |
| `arm_control/arm_control/emergency_stop.py` | 新建 | 急停机制 |
| `arm_vision/arm_vision/vision_node.py` | 新建 | 整合 YOLO+ICP |
| `arm_vision/arm_vision/hand_eye_calibration.py` | 新建 | 手眼标定 |
| `arm_navigation/arm_navigation/motion_planner.py` | 新建 | 运动规划 |
| `vehicle_decision/state_machine.py` | 修改 | 添加 CHARGING 状态 |
| `vehicle_hal/chassis_can_driver.py` | 修改 | 添加 0x300/0x301 |
| `g60pro_gazebo/models/charging_station/` | 新建 | 充电站仿真模型 |
| `g60pro_gazebo/models/ev_simplified/` | 新建 | EV仿真模型 |
| `g60pro_launch/launch/system.launch.py` | 修改 | 添加 arm 相关节点 |

---

## 七、技术风险

| 风险 | 影响 | 应对 |
|------|------|------|
| 机械臂未安装到车上 | 无法做实车测试 | 先做仿真，机械臂安装后快速联调 |
| 手眼标定误差大 | 视觉引导不准 | 使用高精度标定板，多次平均 |
| 力控插枪失败 | 损坏充电口 | 限力 ≤15N，多次检测到位信号 |
| 1个月时间紧张 | 任务无法完成 | 优先保证核心功能（SDK+视觉+力控），非核心可简化 |

---

## 八、里程碑

| 周次 | 里程碑 | 验证方式 |
|------|--------|---------|
| 第1周 | 机械臂可通过 ROS2 控制运动；视觉节点持续输出位姿 | 话题 echo /arm/state |
| 第2周 | 机械臂可从 home 移动到视觉引导的目标点；力控插枪成功 | 实车测试 |
| 第3周 | CAN 协同打通；急停机制可用；Gazebo 仿真可运行 | CAN 回环测试 |
| 第4周 | 全流程联调（仿真+实车）；基本流程可演示 | 演示验收 |

---

## 九、依赖关系（关键路径）

```
Day1: Flexiv SDK → arm_driver.py
Day3: arm_driver → vision_node (订阅位姿)
Day5: vision_node → hand_eye_calibration (标定 T_cam2gripper)
Day6: hand_eye_calibration → motion_planner (路径计算)
Day9: motion_planner → force_controller (力控插枪)
Day11: force_controller → CAN 0x301 (状态上报)
Day11: CAN 0x300 → force_controller (触发开始)
Day13: emergency_stop → 所有节点 (急停覆盖)
Day15: Gazebo 模型 → 全流程仿真
```

---

## 十、与原方案对比（删减内容）

以下内容**不在本月范围内**，可后续迭代：

| 原方案内容 | 删减原因 |
|-----------|---------|
| 云端调度系统 | 同事负责，已基本完成 |
| GB/T 27930 BMS 通信 | 同事负责 |
| 精细停车策略 | 一个月内暂不实现，导航精度先用现有方案 |
| 充电流程状态机扩展 | 简化为 CAN 信号协同 |
| 多车型适配层 | 先用 YOLO 通用检测，后续按需扩展 |
| Gazebo 全流程仿真（详细） | 只搭建基础环境，不追求完整 |
| 能源管理模块 | 同事的云端模块负责 |
