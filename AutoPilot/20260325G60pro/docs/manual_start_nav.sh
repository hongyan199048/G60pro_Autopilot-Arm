#!/bin/bash
# 手动逐步启动 Nav2，用于调试

cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro
source /opt/ros/humble/setup.bash
source install/setup.bash

MAP_YAML="maps/g60pro_v8.yaml"
MAP_PBSTREAM="maps/g60pro_v8.pbstream"

echo "=========================================="
echo "手动逐步启动 Nav2"
echo "=========================================="

# Phase 1: robot_description
echo ""
echo "[1/5] 启动 robot_description..."
ros2 launch robot_description description.launch.py > /tmp/desc.log 2>&1 &
DESC_PID=$!
sleep 2
if ros2 node list 2>/dev/null | grep -q robot_state_publisher; then
    echo "  ✓ robot_state_publisher 运行中 (PID: $DESC_PID)"
else
    echo "  ✗ robot_state_publisher 启动失败"
    exit 1
fi

# Phase 2: 雷达
echo ""
echo "[2/5] 启动雷达..."
ros2 run rslidar_sdk rslidar_sdk_node --ros-args -p config_path:="src/rslidar_sdk/config/config.yaml" > /tmp/rslidar.log 2>&1 &
RSLIDAR_PID=$!
ros2 launch lakibeam1 lakibeam1_g60pro.launch.py > /tmp/lakibeam.log 2>&1 &
LAKIBEAM_PID=$!
sleep 3
if ros2 topic list 2>/dev/null | grep -q "/lidar/rs16/points"; then
    echo "  ✓ Helios16 点云发布中 (PID: $RSLIDAR_PID)"
else
    echo "  ✗ Helios16 未发布点云"
fi
if ros2 topic list 2>/dev/null | grep -q "/lidar/single_1/scan"; then
    echo "  ✓ 单线雷达发布中 (PID: $LAKIBEAM_PID)"
else
    echo "  ✗ 单线雷达未发布"
fi

# Phase 3: Cartographer 纯定位
echo ""
echo "[3/5] 启动 Cartographer 纯定位..."
ros2 launch robot_slam slam_real_localization.launch.py \
  use_sim_time:=false \
  pbstream_file:="$MAP_PBSTREAM" > /tmp/carto.log 2>&1 &
CARTO_PID=$!
sleep 5
if ros2 node list 2>/dev/null | grep -q cartographer; then
    echo "  ✓ Cartographer 运行中 (PID: $CARTO_PID)"
else
    echo "  ✗ Cartographer 启动失败"
    echo "  日志："
    tail -20 /tmp/carto.log
    exit 1
fi

# Phase 4: Nav2
echo ""
echo "[4/5] 启动 Nav2..."
ros2 launch robot_navigation navigation_real.launch.py \
  map:="$MAP_YAML" > /tmp/nav2.log 2>&1 &
NAV2_PID=$!
sleep 5
if ros2 node list 2>/dev/null | grep -q "controller_server\|planner_server"; then
    echo "  ✓ Nav2 运行中 (PID: $NAV2_PID)"
else
    echo "  ✗ Nav2 启动失败"
    echo "  日志："
    tail -20 /tmp/nav2.log
fi

# Phase 5: RViz
echo ""
echo "[5/5] 启动 RViz..."
rviz2 -d src/robot_rviz/rviz/navigation_real.rviz > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!
sleep 2
echo "  ✓ RViz 启动 (PID: $RVIZ_PID)"

echo ""
echo "=========================================="
echo "启动完成！"
echo "=========================================="
echo ""
echo "运行的进程："
echo "  robot_description: $DESC_PID"
echo "  rslidar_sdk:       $RSLIDAR_PID"
echo "  lakibeam1:         $LAKIBEAM_PID"
echo "  Cartographer:      $CARTO_PID"
echo "  Nav2:              $NAV2_PID"
echo "  RViz:              $RVIZ_PID"
echo ""
echo "检查话题："
ros2 topic list 2>/dev/null | grep -E "/map|/tf|/scan|lidar"
echo ""
echo "按 Ctrl+C 停止"

trap "kill $DESC_PID $RSLIDAR_PID $LAKIBEAM_PID $CARTO_PID $NAV2_PID $RVIZ_PID 2>/dev/null; exit" SIGINT
wait