#!/bin/bash
# 测试单线雷达是否能检测到障碍物

echo "========== 实时监控单线雷达 1（右前）数据 =========="
echo "请在右前方（车头右侧 45° 方向，距离 0.5-3m）放置障碍物"
echo "观察 ranges 数组中是否出现有效距离值（非 inf）"
echo ""
ros2 topic echo /lidar/single_1/scan --field ranges | head -50

echo ""
echo "========== 实时监控单线雷达 2（左后）数据 =========="
echo "请在左后方（车尾左侧 135° 方向，距离 0.5-3m）放置障碍物"
echo "观察 ranges 数组中是否出现有效距离值（非 inf）"
echo ""
ros2 topic echo /lidar/single_2/scan --field ranges | head -50

echo ""
echo "========== 检查雷达扫描范围配置 =========="
echo "右前雷达（192.168.2.151）："
echo "  - 扫描范围：45° ~ 315°（270° 扇区）"
echo "  - 安装朝向：车头右前 -45°（URDF yaw）"
echo "  - 有效检测区域：车体右前象限"
echo ""
echo "左后雷达（192.168.2.150）："
echo "  - 扫描范围：45° ~ 315°（270° 扇区）"
echo "  - 安装朝向：车尾左后 135°（URDF yaw）"
echo "  - 有效检测区域：车体左后象限"