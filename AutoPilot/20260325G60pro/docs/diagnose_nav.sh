#!/bin/bash
# 诊断 Nav2 启动问题

cd /home/admin123/Development/G60Pro/AutoPilot/20260325G60pro
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "Nav2 启动诊断"
echo "=========================================="
echo ""

# 1. 检查地图文件
echo "[1/8] 检查地图文件..."
if [ -f "maps/g60pro_v8.yaml" ]; then
    echo "  ✓ g60pro_v8.yaml 存在"
else
    echo "  ✗ g60pro_v8.yaml 不存在"
fi

if [ -f "maps/g60pro_v8.pbstream" ]; then
    echo "  ✓ g60pro_v8.pbstream 存在"
else
    echo "  ✗ g60pro_v8.pbstream 不存在"
fi

# 2. 检查 Helios16 网络连接
echo ""
echo "[2/8] 检查 Helios16 雷达网络（192.168.2.200）..."
if ping -c 1 -W 1 192.168.2.200 > /dev/null 2>&1; then
    echo "  ✓ Helios16 可达"
else
    echo "  ✗ Helios16 不可达（检查网络配置）"
fi

# 3. 检查单线雷达网络连接
echo ""
echo "[3/8] 检查单线雷达网络..."
if ping -c 1 -W 1 192.168.2.151 > /dev/null 2>&1; then
    echo "  ✓ 右前雷达（192.168.2.151）可达"
else
    echo "  ✗ 右前雷达（192.168.2.151）不可达"
fi

if ping -c 1 -W 1 192.168.2.150 > /dev/null 2>&1; then
    echo "  ✓ 左后雷达（192.168.2.150）可达"
else
    echo "  ✗ 左后雷达（192.168.2.150）不可达"
fi

# 4. 检查 CAN 总线
echo ""
echo "[4/8] 检查 CAN 总线..."
if ip link show can0 > /dev/null 2>&1; then
    CAN_STATE=$(ip -details link show can0 | grep -oP 'state \K\w+')
    echo "  ✓ can0 存在，状态: $CAN_STATE"
else
    echo "  ✗ can0 不存在"
fi

# 5. 测试 robot_description
echo ""
echo "[5/8] 测试 robot_description 启动..."
timeout 5 ros2 launch robot_description description.launch.py > /tmp/desc_test.log 2>&1 &
DESC_PID=$!
sleep 3

if ros2 node list 2>/dev/null | grep -q robot_state_publisher; then
    echo "  ✓ robot_state_publisher 启动成功"
    kill $DESC_PID 2>/dev/null
else
    echo "  ✗ robot_state_publisher 启动失败"
    echo "  日志："
    tail -10 /tmp/desc_test.log
    kill $DESC_PID 2>/dev/null
fi

# 6. 测试 lakibeam1
echo ""
echo "[6/8] 测试双单线雷达启动..."
timeout 5 ros2 launch lakibeam1 lakibeam1_g60pro.launch.py > /tmp/lakibeam_test.log 2>&1 &
LAKI_PID=$!
sleep 3

if ros2 topic list 2>/dev/null | grep -q "/lidar/single_1/scan"; then
    echo "  ✓ 单线雷达话题发布成功"
    kill $LAKI_PID 2>/dev/null
else
    echo "  ✗ 单线雷达话题未发布"
    echo "  日志："
    tail -10 /tmp/lakibeam_test.log
    kill $LAKI_PID 2>/dev/null
fi

# 7. 检查 slam_real_localization.launch.py 是否存在
echo ""
echo "[7/8] 检查 Cartographer 纯定位 launch 文件..."
if [ -f "src/robot_slam/launch/slam_real_localization.launch.py" ]; then
    echo "  ✓ slam_real_localization.launch.py 存在"
elif [ -f "install/robot_slam/share/robot_slam/launch/slam_real_localization.launch.py" ]; then
    echo "  ✓ slam_real_localization.launch.py 已安装"
else
    echo "  ✗ slam_real_localization.launch.py 不存在"
    echo "  可用的 launch 文件："
    ls -1 src/robot_slam/launch/*.launch.py 2>/dev/null || echo "    无"
fi

# 8. 检查 navigation_real.launch.py 是否存在
echo ""
echo "[8/8] 检查 Nav2 launch 文件..."
if [ -f "src/robot_navigation/launch/navigation_real.launch.py" ]; then
    echo "  ✓ navigation_real.launch.py 存在"
elif [ -f "install/robot_navigation/share/robot_navigation/launch/navigation_real.launch.py" ]; then
    echo "  ✓ navigation_real.launch.py 已安装"
else
    echo "  ✗ navigation_real.launch.py 不存在"
    echo "  可用的 launch 文件："
    ls -1 src/robot_navigation/launch/*.launch.py 2>/dev/null || echo "    无"
fi

echo ""
echo "=========================================="
echo "诊断完成"
echo "=========================================="