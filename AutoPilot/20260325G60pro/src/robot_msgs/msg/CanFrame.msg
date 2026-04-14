# CAN 数据帧

# CAN ID (11位标准帧或29位扩展帧)
uint32 can_id

# 数据长度 (0-8)
uint8 dlc

# 数据 (8字节)
uint8[8] data

# 时间戳
builtin_interfaces/Time stamp