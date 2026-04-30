#!/bin/bash
# 诊断单线雷达 costmap 问题
# 用法: ./diagnose_single_lidar_costmap.sh

echo "=========================================="
echo "单线雷达 Costmap 诊断脚本"
echo "=========================================="

echo ""
echo "【1】检查单线雷达话题是否存在"
echo "-------------------------------------------"
ros2 topic list | grep -E "(single_1|single_2)"

echo ""
echo "【2】检查单线雷达话题频率（5秒）"
echo "-------------------------------------------"
timeout 5 ros2 topic hz /lidar/single_1/scan 2>&1 | head -n 5 &
timeout 5 ros2 topic hz /lidar/single_2/scan 2>&1 | head -n 5 &
wait

echo ""
echo "【3】检查单线雷达数据内容（1条消息）"
echo "-------------------------------------------"
echo "--- single_1 ---"
timeout 2 ros2 topic echo /lidar/single_1/scan --once 2>&1 | head -n 20
echo ""
echo "--- single_2 ---"
timeout 2 ros2 topic echo /lidar/single_2/scan --once 2>&1 | head -n 20

echo ""
echo "【4】检查 TF 树中是否存在单线雷达 frame"
echo "-------------------------------------------"
ros2 run tf2_ros tf2_echo map single_lidar_1_link 2>&1 | head -n 10 &
sleep 2
ros2 run tf2_ros tf2_echo map single_lidar_2_link 2>&1 | head -n 10 &
sleep 2

echo ""
echo "【5】检查 costmap 是否收到单线雷达数据"
echo "-------------------------------------------"
echo "查看 local_costmap 日志（Ctrl+C 停止）："
ros2 topic echo /local_costmap/costmap_updates --once 2>&1 | head -n 30

echo ""
echo "=========================================="
echo "诊断完成！请检查以上输出"
echo "=========================================="
