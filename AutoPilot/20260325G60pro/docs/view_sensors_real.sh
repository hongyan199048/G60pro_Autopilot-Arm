#!/bin/bash
# G60Pro 传感器查看脚本
# 启动所有传感器驱动 + RViz，用于观测外设状态
# 不跑 SLAM，不跑导航

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/.."

echo "=========================================="
echo "G60Pro 传感器查看模式"
echo "=========================================="

# 清理残留进程
echo "[1/4] 清理残留进程..."
pkill -9 -f rslidar_sdk_node      2>/dev/null || true
pkill -9 -f lakibeam1_scan_node   2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f rviz2                 2>/dev/null || true
sleep 1

# 加载 ROS2 环境
echo "[2/4] 加载 ROS2 环境..."
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# 启动机器人描述（TF 树）
echo "[3/4] 启动机器人描述节点（TF）..."
ros2 launch robot_description description.launch.py &
DESCRIPTION_PID=$!
sleep 2

# 启动 Helios16 多线激光雷达
echo "[4/4] 启动传感器驱动..."
ros2 run rslidar_sdk rslidar_sdk_node \
  --ros-args \
  -p config_path:="$WS_DIR/src/rslidar_sdk/config/config.yaml" &
RS16_PID=$!

# 启动双 LakiBeam1S 单线激光雷达
ros2 launch lakibeam1 lakibeam1_g60pro.launch.py &
LAKI_PID=$!

# TODO: Orbbec 相机（驱动就绪后取消注释）
# ros2 launch orbbec_camera dabai_dcw2.launch.py &
# ORBBEC_PID=$!

sleep 3

# 启动 RViz
echo "启动 RViz..."
rviz2 -d "$WS_DIR/src/robot_rviz/rviz/sensors_view_real.rviz" &
RVIZ_PID=$!

echo ""
echo "=========================================="
echo "传感器查看模式已启动！"
echo ""
echo "进程 PID:"
echo "  - robot_description: $DESCRIPTION_PID"
echo "  - Helios16 (rs16):   $RS16_PID"
echo "  - LakiBeam1S x2:     $LAKI_PID"
echo ""
echo "RViz 显示内容："
echo "  白色点云  -> Helios16 多线 /lidar/rs16/points"
echo "  橙色扫描  -> 单线雷达右前 /lidar/single_1/scan"
echo "  绿色扫描  -> 单线雷达左后 /lidar/single_2/scan"
echo ""
echo "按 Ctrl+C 停止所有节点"
echo "=========================================="

trap "echo '正在停止所有节点...'; kill $DESCRIPTION_PID $RS16_PID $LAKI_PID $RVIZ_PID 2>/dev/null; exit 0" SIGINT SIGTERM

wait
