#!/bin/bash
# G60Pro Gazebo 导航测试脚本
#
# 流程说明:
#   Phase 1: SLAM 建图（使用键盘控制机器人行走建图）
#   Phase 2: 保存地图
#   Phase 3: Nav2 导航（使用保存的地图自主导航）
#
# 使用方式:
#   ./start_nav_sim.sh              # 交互式（每步暂停确认）
#   ./start_nav_sim.sh auto         # 自动执行（用于脚本集成）

INTERACTIVE=true
if [ "$1" = "auto" ]; then
    INTERACTIVE=false
fi

WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAP_BASE="g60pro_sim_map"
MAP_DIR="${WORKSPACE_DIR}/maps"

# 自动查找下一个可用版本号 v0, v1, v2 ...
_ver=0
while [ -f "${MAP_DIR}/${MAP_BASE}_v${_ver}.yaml" ]; do
    _ver=$(( _ver + 1 ))
done
MAP_NAME="${MAP_BASE}_v${_ver}"
MAP_YAML="${MAP_DIR}/${MAP_NAME}.yaml"

function pause() {
    if [ "$INTERACTIVE" = "true" ]; then
        echo ""
        read -p "按 Enter 继续..." key
    fi
}

function cleanup() {
    echo "[清理] 停止残留进程..."
    pkill -9 -f gzserver 2>/dev/null
    pkill -9 -f gazebo 2>/dev/null
    pkill -9 -f spawn 2>/dev/null
    pkill -9 -f cartographer 2>/dev/null
    pkill -9 -f rviz2 2>/dev/null
    pkill -9 -f robot_base 2>/dev/null
    pkill -9 -f map_saver 2>/dev/null
    sleep 2
}

function header() {
    echo ""
    echo "================================================="
    echo "  G60Pro Gazebo 导航测试"
    echo "================================================="
}

# ========== Phase 0: 清理环境 ==========
header
echo "Phase 0: 清理残留进程"
cleanup

# ========== Phase 1: 编译 ==========
header
echo "Phase 1: 编译项目"
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build
if [ $? -ne 0 ]; then
    echo "[错误] 编译失败！"
    exit 1
fi
echo "[OK] 编译完成"

# ========== Phase 2: 启动 Gazebo ==========
header
echo "Phase 2: 启动 Gazebo 仿真"
echo "  - 机器人描述 + URDF"
echo "  - Gazebo 物理世界"
echo "  - planar_move 插件（接收 /cmd_vel，提供 /odom）"
ros2 launch robot_gazebo sim.launch.py &
GAZEBOPID=$!
echo "  Gazebo PID: $GAZEBOPID"
echo "  等待 Gazebo 启动..."
sleep 10

# 检查 Gazebo 是否还在运行
if ! kill -0 $GAZEBOPID 2>/dev/null; then
    echo "[错误] Gazebo 启动失败（进程已退出）"
    exit 1
fi
echo "[OK] Gazebo 已启动"

# ========== Phase 3: 启动底盘节点 ==========
header
echo "Phase 3: 启动底盘节点"
echo "  - 订阅 /cmd_vel → 发布 /odom + TF (odom → base_footprint)"
ros2 launch robot_base base.launch.py use_sim_time:=true use_sim:=true &
BASEPID=$!
echo "  robot_base PID: $BASEPID"
sleep 3

# ========== Phase 4: 启动 Cartographer SLAM ==========
header
echo "Phase 4: 启动 Cartographer SLAM"
echo "  - 订阅 /lidar/multi/points (Helios16 点云)"
echo "  - 发布 /map (占据栅格地图)"
echo "  - 发布 /scan (2D 扫描，用于 Nav2 避障)"
ros2 launch robot_slam slam.launch.py use_sim_time:=true &
SLAMPID=$!
echo "  Cartographer PID: $SLAMPID"
sleep 5

# ========== Phase 5: 启动 RViz (建图视图) ==========
header
echo "Phase 5: 启动 RViz（SLAM 建图视图）"
echo "  显示: /map, /lidar/multi/points, TF, RobotModel"
ros2 run rviz2 rviz2 -d "${WORKSPACE_DIR}/src/robot_rviz/rviz/slam_sim.rviz" &
RVIZPID=$!
echo "  RViz PID: $RVIZPID"

pause

# ========== Phase 6: 键盘控制建图 ==========
header
echo "Phase 6: 键盘控制机器人行走建图"
echo ""
echo "请在新终端中运行以下命令来控制机器人："
echo ""
echo "  cd $WORKSPACE_DIR"
echo "  source /opt/ros/humble/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/cmd_vel"
echo ""
echo "键盘控制说明："
echo "  i/I  - 前进/后退"
echo "  j/l  - 左转/右转"
echo "  k    - 停止"
echo "  u/o  - 左/右平移"
echo ""
echo "用键盘控制机器人在 Gazebo 中行走，覆盖整个工作区域以完成建图。"
echo ""
if [ "$INTERACTIVE" = "true" ]; then
    pause
fi

# ========== Phase 7: 保存地图 ==========
header
echo "Phase 7: 保存 Cartographer 地图"
echo "保存为: ${MAP_DIR}/${MAP_NAME}.yaml"
echo ""
echo "版本号自动递增，已有版本不会被覆盖。"
echo ""

# 使用 nav2_map_server 保存 Cartographer 发布的 /map
# 前提：Cartographer occupancy_grid_node 必须正在运行并发布 /map
ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_NAME}" &
SAVERPID=$!
echo "  地图保存进程 PID: $SAVERPID"
sleep 5
kill $SAVERPID 2>/dev/null

if [ -f "${MAP_DIR}/${MAP_NAME}.yaml" ]; then
    echo "[OK] 地图已保存: ${MAP_DIR}/${MAP_NAME}.yaml"
else
    echo "[警告] 地图未生成，检查 Cartographer 是否在运行..."
fi

pause

# ========== Phase 8: 停止 SLAM，重新启动 Nav2 导航 ==========
header
echo "Phase 8: 停止 SLAM，准备启动 Nav2 导航"
echo "停止 Cartographer..."
kill $SLAMPID 2>/dev/null
sleep 2

# 停止 RViz
kill $RVIZPID 2>/dev/null
sleep 1

# ========== Phase 9: 启动 Nav2 导航 ==========
header
echo "Phase 9: 启动 Nav2 导航"
echo ""
echo "启动内容："
echo "  - AMCL 定位（使用保存的地图）"
echo "  - Nav2 Planner（全局路径规划）"
echo "  - Nav2 Controller（DWA 局部规划）"
echo "  - 全局/局部代价地图（避障）"
echo "  - EKF 定位融合"
echo ""
ros2 launch robot_navigation navigation.launch.py use_sim_time:=true map:="${MAP_YAML}" &
NAVPID=$!
echo "  Nav2 导航 PID: $NAVPID"
sleep 5

# ========== Phase 10: 启动 RViz（导航视图） ==========
header
echo "Phase 10: 启动 RViz（导航视图）"
echo "显示: /map, /scan, /plan (全局路径), /local_plan (局部路径)"
ros2 run rviz2 rviz2 -d "${WORKSPACE_DIR}/src/robot_rviz/rviz/navigation_sim.rviz" &
NAV_RVIZ_PID=$!
echo "  RViz PID: $NAV_RVIZ_PID"

sleep 3

# ========== 完成 ==========
header
echo "================================================="
echo "  导航已启动！"
echo "================================================="
echo ""
echo "当前运行的进程："
echo "  - Gazebo:         $GAZEBOPID"
echo "  - robot_base:     $BASEPID"
echo "  - Nav2 导航:      $NAVPID"
echo "  - RViz (导航):    $NAV_RVIZ_PID"
echo ""
echo "保存的地图："
echo "  ${MAP_YAML}"
echo ""
echo "RViz 操作说明："
echo "  1. 先点击 '2D Pose Estimate' 按钮，在地图上点击机器人当前位置并拖拽设置朝向（初始化定位）"
echo "  2. 再点击 '2D Goal Pose' 按钮，在地图上点击目标点并拖拽设置朝向"
echo "  3. 机器人将自主规划路径并导航到目标点"
echo ""
echo "按 Ctrl+C 停止所有节点"
echo "================================================="

# 清理函数
function shutdown() {
    echo ""
    echo "[关闭] 停止所有节点..."
    kill $GAZEBOPID $BASEPID $NAVPID $NAV_RVIZ_PID 2>/dev/null
    cleanup
    echo "[完成]"
    exit 0
}
trap shutdown SIGINT

wait
