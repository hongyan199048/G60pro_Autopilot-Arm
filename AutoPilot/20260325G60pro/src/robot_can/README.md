# robot_can

**控制层 — CAN 总线通信**

## 功能

基于 DBC 协议的 CAN 总线通信节点，负责工控机与 RDM 底盘控制器之间的双向通信。

## 核心节点

### `can_node`

**功能**：
- 订阅 `/motor_cmd`，编码为 CAN 帧，通过 CAN1 发送到 RDM 控制器
- 接收 CAN4 反馈帧，解码为电机状态，发布 `/motor_state`
- 基于 DBC 协议（V1.0）精确编解码

---

## CAN 总线配置

### CAN1（can0）— 控制指令

- **波特率**：500 kbps
- **方向**：工控机 → RDM 控制器
- **发送频率**：50 Hz（20ms 周期）

**发送帧**：
| CAN ID | 名称 | 内容 |
|--------|------|------|
| `0x210` | `LAS_Fr01` | 四轮驱动转速指令（RPM） |
| `0x211` | `LAS_Fr02` | 四轮转向角度指令（度） |
| `0x212` | `LAS_Fr03` | 任务状态帧（滚动计数器、故障码） |

### CAN4（can1）— 状态反馈

- **波特率**：1 Mbps
- **方向**：RDM 控制器 → 工控机

**接收帧**：
| CAN ID | 名称 | 内容 |
|--------|------|------|
| `0x19` | `RDM_Fr35` | 四轮转速反馈（RPM） |
| `0x1B` | `RDM_Fr36` | 四轮转向角反馈（度） |
| `0x1F` | `RDM_Fr03` | 整车状态（高压就绪、急停、驻车锁） |
| `0x2A` | `RDM_Fr37` | 电机位置反馈 1（32bit signed） |
| `0x2B` | `RDM_Fr38` | 电机位置反馈 2（32bit signed） |

---

## DBC 协议

**协议文件**：`config/can1/G60_CAN1_RDM_V1.0.dbc`

**版本**：V1.0（2026-04-19 重构，替换 V3.9）

**编解码库**：`cantools`（Python）

---

## 单位转换

| 物理量 | ROS2 单位 | CAN 单位 | 转换公式 |
|--------|----------|---------|---------|
| 转向角 | 弧度（rad） | 度（deg） | `deg = rad × 180 / π` |
| 驱动转速 | 弧度/秒（rad/s） | 转/分（RPM） | `RPM = rad/s × 60 / (2π)` |

---

## 启动命令

### 初始化 CAN 总线

```bash
# CAN1（控制指令）
sudo ip link set can0 up type can bitrate 500000

# CAN4（状态反馈）
sudo ip link set can1 up type can bitrate 1000000
```

### 启动 CAN 节点

```bash
ros2 run robot_can can_node
```

---

## 关键话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/motor_cmd` | MotorCmd | 输入 | 电机指令（4 转向角 + 4 转速） |
| `/motor_state` | MotorState | 输出 | 电机状态反馈（转速、角度、温度、电流） |

---

## 消息定义

### `MotorCmd.msg`

```
float32[4] steer_angle      # 转向角（弧度）
float32[4] drive_velocity   # 驱动转速（rad/s）
builtin_interfaces/Time stamp
```

### `MotorState.msg`

```
float32[4] steer_angle_feedback      # 转向角反馈（弧度）
float32[4] drive_velocity_feedback   # 转速反馈（rad/s）
float32[4] temperature               # 电机温度（°C）
float32[4] current                   # 电机电流（A）
float32 voltage                      # 电机电压（V）
uint16 error_code                    # 故障码
builtin_interfaces/Time stamp
```

---

## 安全机制

### 心跳检测

- **发送频率**：50 Hz（20ms）
- **超时时间**：100ms
- **超时行为**：RDM 控制器自动停车

**注意**：必须持续发送控制指令，否则底盘会自动停止。

### 急停信号

- **CAN ID**：`0x1F`（`RDM_Fr03`）
- **信号**：`RDM_Fr03_EmergencyStop`
- **状态**：0=正常，1=急停

---

## 故障排查

### 1. CAN 总线未初始化

**症状**：`can_node` 启动失败，提示 "Network is down"

**解决**：
```bash
sudo ip link set can0 up type can bitrate 500000
sudo ip link set can1 up type can bitrate 1000000
```

### 2. 底盘无响应

**症状**：发送指令后，底盘不动

**排查**：
1. 检查 CAN 总线是否正常：`candump can0`
2. 检查 RDM 控制器是否上电
3. 检查急停按钮是否按下
4. 检查高压是否就绪（`RDM_Fr03_HighVoltageReady`）

### 3. DBC 解码失败

**症状**：`can_node` 报错 "Failed to decode CAN frame"

**排查**：
1. 检查 DBC 文件版本是否匹配（V1.0）
2. 检查 CAN ID 是否在 DBC 中定义
3. 检查数据长度（DLC）是否正确

---

## 依赖

- `python-can` — CAN 总线接口
- `cantools` — DBC 协议编解码
- `robot_msgs` — 自定义消息（MotorCmd, MotorState）

---

## 注意事项

1. **CAN 总线权限**：需要 root 权限或将用户加入 `dialout` 组
2. **DBC 版本**：必须使用 V1.0，与 RDM 控制器固件匹配
3. **心跳机制**：必须持续发送指令，否则底盘自动停车
4. **单位转换**：注意弧度/度、rad/s/RPM 的转换
5. **实车调试**：当前 CAN 总线尚未在实车上完全调通，底盘反馈数据待验证
