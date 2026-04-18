#!/bin/bash
# G60Pro Gazebo 仿真传感器查看脚本
# 启动 Gazebo 仿真环境，查看所有仿真传感器数据
# 不跑 SLAM，不跑导航

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/.."

echo "=========================================="
echo "G60Pro 仿真传感器查看模式"
echo "=========================================="

# 清理残留进程
echo "[1/4] 清理残留进程..."
pkill -9 -f gzserver           2>/dev/null || true
pkill -9 -f gzclient           2>/dev/null || true
pkill -9 -f spawn_entity       2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f robot_base_node    2>/dev/null || true
pkill -9 -f rviz2              2>/dev/null || true
sleep 2

# 加载 ROS2 环境
echo "[2/4] 加载 ROS2 环境..."
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# 启动 Gazebo（含机器人描述 + spawn，所有仿真传感器插件随之激活）
echo "[3/4] 启动 Gazebo 仿真..."
ros2 launch robot_gazebo sim.launch.py &
GAZEBO_PID=$!
echo "  Gazebo PID: $GAZEBO_PID"
echo "  等待 Gazebo 和传感器插件初始化..."
sleep 10

# 检查 Gazebo 是否正常启动
if ! kill -0 $GAZEBO_PID 2>/dev/null; then
    echo "[错误] Gazebo 启动失败"
    exit 1
fi

# 启动底盘节点（发布 odom → base_footprint TF，RViz 需要）
ros2 launch robot_base base.launch.py use_sim_time:=true use_sim:=true &
BASE_PID=$!
echo "  robot_base PID: $BASE_PID"
sleep 3

# 启动 RViz
echo "[4/4] 启动 RViz..."
rviz2 -d "$WS_DIR/src/robot_rviz/rviz/sensors_view_sim.rviz" &
RVIZ_PID=$!
echo "  RViz PID: $RVIZ_PID"

echo ""
echo "=========================================="
echo "仿真传感器查看模式已启动！"
echo ""
echo "进程 PID:"
echo "  - Gazebo:     $GAZEBO_PID"
echo "  - robot_base: $BASE_PID"
echo "  - RViz:       $RVIZ_PID"
echo ""
echo "RViz 显示内容："
echo "  白色点云  -> Helios16 仿真 /lidar/multi/points"
echo "  橙色扫描  -> 单线雷达右前 /lidar/single_1/scan"
echo "  绿色扫描  -> 单线雷达左后 /lidar/single_2/scan"
echo ""
echo "其他可用仿真话题："
echo "  /imu                    -> IMU 数据"
echo "  /camera/front/color/image_raw  -> 前置相机"
echo "  /camera/rear/color/image_raw   -> 后置相机"
echo "  /odom                   -> 底盘里程计"
echo ""
echo "按 Ctrl+C 停止所有节点"
echo "=========================================="

trap "echo '正在停止所有节点...'; kill $GAZEBO_PID $BASE_PID $RVIZ_PID 2>/dev/null; exit 0" SIGINT SIGTERM

wait
