#!/bin/bash
# G60Pro SLAM 一键启动脚本

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/.."

echo "=========================================="
echo "G60Pro Cartographer SLAM 一键启动"
echo "=========================================="

# 第1步：清理残留进程
echo "[1/5] 清理残留进程..."
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gazebo 2>/dev/null
pkill -9 -f spawn 2>/dev/null
pkill -9 -f cartographer 2>/dev/null
pkill -9 -f rviz2 2>/dev/null
sleep 2

# 第2步：编译项目
echo "[2/5] 编译项目..."
cd "$WS_DIR"
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build

echo ""
echo "=========================================="
echo "编译完成！现在将启动各个节点"
echo "请在新的终端窗口中查看各节点运行状态"
echo "=========================================="
echo ""

# 第3步：启动Gazebo
echo "[3/5] 启动Gazebo仿真..."
ros2 launch robot_gazebo sim.launch.py &
GAZEBO_PID=$!
sleep 8

# 第4步：启动底盘节点（广播 odom -> base_footprint TF）
echo "[4/6] 启动底盘节点..."
ros2 launch robot_base base.launch.py use_sim_time:=true use_sim:=true &
BASE_PID=$!
sleep 3

# 第5步：启动Cartographer
echo "[5/6] 启动Cartographer SLAM..."
ros2 launch robot_slam slam.launch.py use_sim_time:=true &
CARTO_PID=$!
sleep 3

# 第6步：启动RViz
echo "[6/6] 启动RViz..."
rviz2 -d "$WS_DIR/src/robot_rviz/rviz/slam_sim.rviz" &
RVIZ_PID=$!

echo ""
echo "=========================================="
echo "所有节点已启动！"
echo ""
echo "进程 PID:"
echo "  - Gazebo:       $GAZEBO_PID"
echo "  - robot_base:   $BASE_PID"
echo "  - Cartographer: $CARTO_PID"
echo "  - RViz:         $RVIZ_PID"
echo ""
echo "下一步：请打开终端4，运行键盘控制："
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/cmd_vel"
echo ""
echo "按 Ctrl+C 停止所有节点"
echo "=========================================="

# 等待用户中断
trap "echo '正在停止所有节点...'; kill $GAZEBO_PID $BASE_PID $CARTO_PID $RVIZ_PID 2>/dev/null; exit 0" SIGINT

wait