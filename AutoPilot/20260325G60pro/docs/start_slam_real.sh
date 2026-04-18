#!/bin/bash
# G60Pro 实车 SLAM 一键启动脚本
# 适用：Helios16 多线激光雷达 + Cartographer 2D 建图（不依赖 IMU）

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/.."

echo "=========================================="
echo "G60Pro 实车 Cartographer SLAM 启动"
echo "=========================================="

# 第1步：清理残留进程
echo "[1/6] 清理残留进程..."
pkill -9 -f cartographer_node    2>/dev/null || true
pkill -9 -f rslidar_sdk_node     2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f rviz2                2>/dev/null || true
sleep 1

# 第2步：初始化 CAN 总线
echo "[2/6] 初始化 CAN 总线..."
sudo ip link set can0 up type can bitrate 500000  2>/dev/null || echo "  [警告] can0 初始化失败，跳过（不影响建图）"
sudo ip link set can1 up type can bitrate 1000000 2>/dev/null || echo "  [警告] can1 初始化失败，跳过"

# 第3步：source 环境
echo "[3/6] 加载 ROS2 环境..."
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# 第4步：启动机器人描述（URDF + TF 静态变换）
echo "[4/6] 启动机器人描述节点（TF）..."
ros2 launch robot_description description.launch.py &
DESCRIPTION_PID=$!
sleep 3

# 第5步：启动 Helios16 激光雷达驱动
echo "[5/6] 启动 Helios16 激光雷达..."
ros2 run rslidar_sdk rslidar_sdk_node \
  --ros-args \
  -p config_path:="$WS_DIR/src/rslidar_sdk/config/config.yaml" &
LIDAR_PID=$!
sleep 3

# 第6步：启动 Cartographer SLAM
# 话题映射：Cartographer 的 points2 <- /lidar/rs16/points（rslidar_sdk 发布）
echo "[6/6] 启动 Cartographer SLAM..."
SLAM_CONFIG_DIR="$WS_DIR/src/robot_slam/config"
ros2 run cartographer_ros cartographer_node \
  -configuration_directory "$SLAM_CONFIG_DIR" \
  -configuration_basename cartographer_real.lua \
  --ros-args \
  --remap points2:=/lidar/rs16/points &
CARTO_PID=$!
sleep 3

ros2 run cartographer_ros cartographer_occupancy_grid_node \
  --ros-args -p resolution:=0.05 -p publish_period_sec:=1.0 &
GRID_PID=$!
sleep 1

# 启动 RViz
echo "启动 RViz..."
rviz2 -d "$WS_DIR/src/robot_rviz/rviz/slam_real.rviz" &
RVIZ_PID=$!

echo ""
echo "=========================================="
echo "所有节点已启动！"
echo ""
echo "进程 PID:"
echo "  - robot_description: $DESCRIPTION_PID"
echo "  - rslidar_sdk:       $LIDAR_PID"
echo "  - Cartographer:      $CARTO_PID"
echo "  - OccupancyGrid:     $GRID_PID"
echo "  - RViz:              $RVIZ_PID"
echo ""
echo "话题说明："
echo "  /lidar/rs16/points  -> Helios16 点云（rslidar_sdk 发布）"
echo "  /map                -> Cartographer 建图结果"
echo ""
echo "键盘控制（新终端）："
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/cmd_vel"
echo ""
echo "保存地图："
echo "  cd $SCRIPT_DIR && ./save_map.sh"
echo ""
echo "按 Ctrl+C 停止所有节点"
echo "=========================================="

# 捕获退出信号，清理所有子进程
trap "echo '正在停止所有节点...'; kill $DESCRIPTION_PID $LIDAR_PID $CARTO_PID $GRID_PID $RVIZ_PID 2>/dev/null; exit 0" SIGINT SIGTERM

wait
