# 电机状态反馈 - 4轮8驱

# 转向角度反馈 (弧度)
float32[4] steer_angle_feedback

# 驱动转速反馈 (rad/s)
float32[4] drive_velocity_feedback

# 电机温度 (摄氏度)
float32[4] temperature

# 电机电流 (A)
float32[4] current

# 电机电压 (V)
float32 voltage

# 错误码
uint16 error_code

# 时间戳
builtin_interfaces/Time stamp