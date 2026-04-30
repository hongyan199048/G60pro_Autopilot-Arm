#!/bin/bash
# 检查单线雷达数据是否正常
# 用法: ./check_single_lidar.sh

echo "=========================================="
echo "单线雷达数据检查"
echo "=========================================="

echo ""
echo "【1】检查 /lidar/single_1/scan 的 frame_id 和数据"
echo "-------------------------------------------"
ros2 topic echo /lidar/single_1/scan --once | grep -A 5 "header:" | head -n 10
echo ""
ros2 topic echo /lidar/single_1/scan --once | grep -A 3 "ranges:" | head -n 20

echo ""
echo "【2】检查 /lidar/single_2/scan 的 frame_id 和数据"
echo "-------------------------------------------"
ros2 topic echo /lidar/single_2/scan --once | grep -A 5 "header:" | head -n 10
echo ""
ros2 topic echo /lidar/single_2/scan --once | grep -A 3 "ranges:" | head -n 20

echo ""
echo "【3】检查 TF: base_footprint → single_lidar_1_link"
echo "-------------------------------------------"
timeout 3 ros2 run tf2_ros tf2_echo base_footprint single_lidar_1_link 2>&1 | head -n 15

echo ""
echo "【4】检查 TF: base_footprint → single_lidar_2_link"
echo "-------------------------------------------"
timeout 3 ros2 run tf2_ros tf2_echo base_footprint single_lidar_2_link 2>&1 | head -n 15

echo ""
echo "=========================================="
echo "检查完成！"
echo "=========================================="
