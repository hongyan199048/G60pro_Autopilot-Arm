# 电机控制指令 - 4轮8驱
# 转向角度: -90° ~ +90° (弧度)
# 驱动转速: RPM 或 rad/s

# 4个轮子的转向角度 (弧度)
float32[4] steer_angle

# 4个轮子的驱动转速 (rad/s)
float32[4] drive_velocity

# 时间戳
builtin_interfaces/Time stamp