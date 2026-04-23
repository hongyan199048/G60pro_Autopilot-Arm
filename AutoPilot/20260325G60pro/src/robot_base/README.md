# robot_base

**控制层 — 底盘运动学解算**

## 功能

4 轮 8 驱全向移动底盘的运动学逆解算节点，将速度指令转换为电机控制指令。

## 核心节点

### `robot_base_node`

**功能**：
- 订阅 `/cmd_vel`（线速度 + 角速度）
- 计算 4 轮 8 驱运动学逆解
- 发布 `/motor_cmd`（4 个转向角 + 4 个驱动转速）
- 发布 `/odom`（开环里程计，速度积分）
- 发布 TF：`odom → base_footprint`（可选）

---

## 运动学模型

### 全向移动（Swerve Drive）

每个轮子独立驱动 + 独立转向，可实现任意方向平移 + 原地旋转。

**输入**：
- `vx` — 前后方向线速度（m/s）
- `vy` — 左右方向线速度（m/s）
- `omega` — 旋转角速度（rad/s）

**输出**（每个轮子）：
- `steer_angle` — 转向角（弧度）
- `drive_velocity` — 驱动转速（rad/s）

**计算公式**：
```python
# 轮子位置：[x, y]（相对车身中心）
# FL: [Lx, Ly], FR: [Lx, -Ly], RL: [-Lx, Ly], RR: [-Lx, -Ly]

# 每个轮子的速度分量
v_wheel_x = vx - omega * y
v_wheel_y = vy + omega * x

# 转向角和驱动速度
steer_angle = atan2(v_wheel_y, v_wheel_x)
drive_velocity = sqrt(v_wheel_x² + v_wheel_y²) / wheel_radius
```

---

## 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `wheel_radius` | 0.15 m | 轮子半径 |
| `wheelbase_x` | 0.61 m | 前后轮距之半 |
| `wheelbase_y` | 0.2705 m | 左右轮距之半 |
| `max_velocity` | 1.0 m/s | 最大线速度 |
| `max_angular` | 1.0 rad/s | 最大角速度 |
| `publish_tf` | true/false | 是否发布 odom→base_footprint TF |

**注意**：
- 实车纯定位模式下，`publish_tf` 必须设为 `false`，避免与 Cartographer 的 TF 冲突
- 仿真模式下，`publish_tf` 设为 `true`（Gazebo 不发布此 TF）

---

## 启动命令

### 仿真模式

```bash
ros2 run robot_base robot_base_node \
  --ros-args -p use_sim:=true -p publish_tf:=true
```

### 实车模式

```bash
# 纯定位模式（不发布 TF）
ros2 run robot_base robot_base_node \
  --ros-args -p use_sim:=false -p publish_tf:=false

# 建图模式（发布 TF）
ros2 run robot_base robot_base_node \
  --ros-args -p use_sim:=false -p publish_tf:=true
```

---

## 关键话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/cmd_vel` | Twist | 输入 | 速度指令（vx, vy, omega） |
| `/motor_cmd` | MotorCmd | 输出 | 电机指令（4 转向角 + 4 转速） |
| `/odom` | Odometry | 输出 | 里程计（开环，速度积分） |
| TF: `odom → base_footprint` | — | 输出 | 里程计坐标系（可选） |

---

## 里程计说明

**类型**：开环里程计（速度积分）

**精度**：低（累积误差大）

**用途**：
- 仅供 Nav2 局部代价地图使用（短时间内精度可接受）
- 不用于全局定位（由 Cartographer 提供）

**数据流**：
```
/cmd_vel → robot_base_node → 速度积分 → /odom
```

**注意**：实车底盘反馈的真实里程计（通过 CAN 总线）尚未集成，当前使用的是指令速度积分。

---

## 配置文件

`config/base.yaml` — 底盘参数配置

**注意**：代码中硬编码的轮距参数（0.61 / 0.2705）会覆盖 YAML 文件中的值。

---

## 依赖

- `robot_msgs` — 自定义消息（MotorCmd）
- `robot_can` — CAN 总线通信（接收 motor_cmd）
- `geometry_msgs` — Twist 消息
- `nav_msgs` — Odometry 消息
- `tf2_ros` — TF 广播

---

## 注意事项

1. **轮距参数**：当前代码中硬编码为 0.61 / 0.2705，与 `base.yaml` 不一致，建议统一
2. **TF 冲突**：实车纯定位模式下，必须设置 `publish_tf:=false`
3. **里程计精度**：开环里程计误差大，不能用于长距离定位
4. **CAN 反馈**：未来应集成底盘真实里程计反馈，替换速度积分
