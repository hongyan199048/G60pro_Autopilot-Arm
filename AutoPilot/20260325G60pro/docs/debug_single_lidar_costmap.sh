#!/bin/bash
# 诊断单线雷达为何无法在 costmap 中生成障碍物

echo "========== 1. 检查单线雷达话题是否发布 =========="
timeout 2 ros2 topic hz /lidar/single_1/scan
timeout 2 ros2 topic hz /lidar/single_2/scan

echo ""
echo "========== 2. 检查单线雷达数据内容 =========="
timeout 2 ros2 topic echo /lidar/single_1/scan --once | head -30

echo ""
echo "========== 3. 检查 TF 树（single_lidar_*_link 是否存在）=========="
ros2 run tf2_tools view_frames

echo ""
echo "========== 4. 检查 local_costmap 是否接收到单线雷达数据 =========="
echo "查看 /local_costmap/costmap_updates 话题（应该有障碍物更新）"
timeout 3 ros2 topic echo /local_costmap/costmap_updates --once

echo ""
echo "========== 5. 检查单线雷达点的高度（相对 base_footprint）=========="
echo "运行 RViz，添加 /lidar/single_1/scan，Fixed Frame=base_footprint，观察点云 Z 坐标"
echo "如果 Z < -0.5m，会被 min_obstacle_height 过滤"

echo ""
echo "========== 6. 启用 Nav2 调试日志 =========="
echo "ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args --log-level debug"
