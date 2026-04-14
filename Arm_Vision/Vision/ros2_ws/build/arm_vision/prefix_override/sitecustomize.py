import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/admin123/Development/G60Pro/Arm_Vision/Vision/ros2_ws/install/arm_vision'
