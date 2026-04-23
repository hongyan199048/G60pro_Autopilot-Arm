# robot_msgs

**自定义消息定义**

## 功能

定义 G60Pro 项目专用的 ROS2 消息类型。

## 消息列表

### 1. `MotorCmd.msg` — 电机控制指令

**用途**：`robot_base_node` → `can_node`

**定义**：
```
float32[4] steer_angle      # 转向角（弧度）
float32[4] drive_velocity   # 驱动转速（rad/s）
builtin_interfaces/Time stamp
```

**说明**：
- `steer_angle[0~3]` — 四个轮子的转向角（FL, FR, RL, RR）
- `drive_velocity[0~3]` — 四个轮子的驱动转速（FL, FR, RL, RR）
- 单位：弧度（rad）、弧度/秒（rad/s）

---

### 2. `MotorState.msg` — 电机状态反馈

**用途**：`can_node` → 监控节点

**定义**：
```
float32[4] steer_angle_feedback      # 转向角反馈（弧度）
float32[4] drive_velocity_feedback   # 转速反馈（rad/s）
float32[4] temperature               # 电机温度（°C）
float32[4] current                   # 电机电流（A）
float32 voltage                      # 电机电压（V）
uint16 error_code                    # 故障码
builtin_interfaces/Time stamp
```

**说明**：
- 反馈数据来自 CAN 总线（RDM 控制器）
- `error_code` — 故障码（0=正常）

---

### 3. `CanFrame.msg` — CAN 帧（原始数据）

**用途**：CAN 总线调试

**定义**：
```
uint32 can_id       # CAN ID
uint8 dlc           # 数据长度（0~8）
uint8[] data        # 数据字节
builtin_interfaces/Time stamp
```

**说明**：
- 用于 CAN 总线原始数据记录和调试
- 当前未在主流程中使用

---

## 编译

```bash
cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro
colcon build --packages-select robot_msgs
source install/setup.bash
```

---

## 使用示例

### Python

```python
from robot_msgs.msg import MotorCmd, MotorState

# 发布电机指令
motor_cmd = MotorCmd()
motor_cmd.steer_angle = [0.0, 0.0, 0.0, 0.0]
motor_cmd.drive_velocity = [1.0, 1.0, 1.0, 1.0]
motor_cmd.stamp = self.get_clock().now().to_msg()
self.publisher.publish(motor_cmd)

# 订阅电机状态
def motor_state_callback(msg: MotorState):
    print(f"温度: {msg.temperature}")
    print(f"电流: {msg.current}")
```

---

## 依赖

- `builtin_interfaces` — ROS2 标准消息（Time）
- `rosidl_default_generators` — 消息生成器

---

## 注意事项

1. **单位统一**：所有角度使用弧度（rad），速度使用 rad/s
2. **数组长度**：`steer_angle` 和 `drive_velocity` 固定长度为 4（四个轮子）
3. **时间戳**：所有消息包含 `stamp` 字段，用于时间同步
