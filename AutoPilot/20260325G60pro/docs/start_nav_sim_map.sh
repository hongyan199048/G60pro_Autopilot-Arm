#!/bin/bash
# G60Pro Gazebo 导航 — 直接导入已有地图（跳过 SLAM 建图）
#
# 与 start_nav_sim.sh 的区别：
#   - 不启动 Cartographer SLAM 建图
#   - 不运行键盘建图阶段
#   - 直接用已有 PGM+YAML 地图 + Cartographer 纯定位（无 AMCL）
#
# 定位方案：Cartographer 纯定位（加载 .pbstream）+ map→base_footprint 直连
# 地图来源：map_server 加载 PGM+YAML 静态地图
# 避障扫描：pointcloud_to_laserscan（Helios16 → /scan）
#
# 用法:
#   ./start_nav_sim_map.sh                  # 使用默认地图 g60pro_v10
#   ./start_nav_sim_map.sh g60pro_v9        # 指定地图名

WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAP_DIR="${WORKSPACE_DIR}/maps"
MAP_NAME="${1:-g60pro_v10}"
MAP_YAML="${MAP_DIR}/${MAP_NAME}.yaml"
PBSTREAM_FILE="${MAP_DIR}/${MAP_NAME}.pbstream"

if [ ! -f "$MAP_YAML" ]; then
    echo "[错误] 地图文件不存在: $MAP_YAML"
    echo "可用地图:"
    ls -1 "${MAP_DIR}"/*.yaml 2>/dev/null | while read f; do
        echo "  $(basename "$f" .yaml)"
    done
    exit 1
fi

if [ ! -f "$PBSTREAM_FILE" ]; then
    echo "[错误] pbstream 文件不存在: $PBSTREAM_FILE"
    echo "纯定位模式需要 .pbstream 文件，请先用 ./start_slam_sim.sh 建图并保存"
    exit 1
fi

NAV2_PARAMS="${WORKSPACE_DIR}/src/robot_navigation/config/nav2_params_sim.yaml"

function cleanup() {
    echo "[清理] 停止残留进程..."
    pkill -9 -f gzserver 2>/dev/null
    pkill -9 -f gazebo 2>/dev/null
    pkill -9 -f rviz2 2>/dev/null
    pkill -9 -f robot_base 2>/dev/null
    pkill -9 -f cartographer 2>/dev/null
    pkill -9 -f pointcloud_to_laserscan 2>/dev/null
    sleep 2
}

function header() {
    echo ""
    echo "================================================="
    echo "  G60Pro Gazebo 导航（已有地图 + Cartographer 纯定位）"
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
echo "  - 机器人模型 + warehouse 世界"
echo "  - planar_move 插件（/cmd_vel → /odom，不发布 TF）"
ros2 launch robot_gazebo sim.launch.py &
GAZEBOPID=$!
echo "  Gazebo PID: $GAZEBOPID"
sleep 10

if ! kill -0 $GAZEBOPID 2>/dev/null; then
    echo "[错误] Gazebo 启动失败"
    exit 1
fi
echo "[OK] Gazebo 已启动"

# ========== Phase 3: 启动底盘节点 ==========
header
echo "Phase 3: 启动底盘节点"
echo "  - 订阅 /cmd_vel → 运动学解算 → CAN 指令（仿真模式不实际发送）"
ros2 launch robot_base base.launch.py use_sim_time:=true use_sim:=true &
BASEPID=$!
echo "  robot_base PID: $BASEPID"
sleep 3

# ========== Phase 4: 启动 Cartographer 纯定位 ==========
header
echo "Phase 4: 启动 Cartographer 纯定位"
echo "  - 加载 pbstream: ${PBSTREAM_FILE}"
echo "  - 发布 map→base_footprint TF（定位）"
echo "  - occupancy_grid_node 发布 /map（从 pbstream 直接生成）"
ros2 launch robot_slam slam_sim_localization.launch.py \
    pbstream_file:="${PBSTREAM_FILE}" &
SLAMPID=$!
echo "  Cartographer PID: $SLAMPID"
sleep 8

# 检查 Cartographer 是否启动成功
if ! kill -0 $SLAMPID 2>/dev/null; then
    echo "[错误] Cartographer 启动失败"
    exit 1
fi
echo "[OK] Cartographer 纯定位已启动"

# ========== Phase 5: 启动 pointcloud_to_laserscan ==========
header
echo "Phase 5: 启动 pointcloud_to_laserscan"
echo "  - Helios16 3D 点云 → 2D /scan（供代价地图避障）"
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
    --ros-args \
    -p use_sim_time:=true \
    -p target_frame:=base_footprint \
    -p transform_tolerance:=0.5 \
    -p min_height:=0.1 \
    -p max_height:=2.0 \
    -p angle_min:=-3.14159 \
    -p angle_max:=3.14159 \
    -p angle_increment:=0.00436 \
    -p scan_time:=0.1 \
    -p range_min:=1.6 \
    -p range_max:=30.0 \
    -p use_inf:=true \
    -r cloud_in:=/lidar/multi/points \
    -r scan:=/scan &
SCANPID=$!
echo "  pointcloud_to_laserscan PID: $SCANPID"
sleep 3

# ========== Phase 6: 启动 Nav2 导航（无 AMCL） ==========
header
echo "Phase 6: 启动 Nav2 导航（无 AMCL，Cartographer 提供定位 + /map）"
echo ""
echo "启动内容："
echo "  - Nav2 Planner + Controller (DWA)"
echo "  - 全局/局部代价地图"
echo "  - /map 由 Cartographer occupancy_grid_node 提供（无需 map_server）"
echo "  - 无 AMCL（定位由 Cartographer map→base_footprint 提供）"
echo ""
ros2 launch robot_navigation nav2_no_amcl.launch.py \
    params_file:="${NAV2_PARAMS}" \
    use_sim_time:=true &
NAVPID=$!
echo "  Nav2 PID: $NAVPID"
sleep 8

# ========== Phase 7: 启动 RViz（导航视图） ==========
header
echo "Phase 7: 启动 RViz（导航视图）"
ros2 run rviz2 rviz2 -d "${WORKSPACE_DIR}/src/robot_rviz/rviz/navigation_sim.rviz" &
RVIZPID=$!
echo "  RViz PID: $RVIZPID"
sleep 3

# ========== 完成 ==========
header
echo "================================================="
echo "  导航已启动！（地图: ${MAP_NAME}）"
echo "================================================="
echo ""
echo "运行中的进程："
echo "  - Gazebo:              $GAZEBOPID"
echo "  - robot_base:          $BASEPID"
echo "  - Cartographer 纯定位: $SLAMPID"
echo "  - pointcloud_to_lscan: $SCANPID"
echo "  - Nav2 导航:           $NAVPID"
echo "  - RViz:                $RVIZPID"
echo ""
echo "TF 树：map → base_footprint → base_link → ..."
echo "定位由 Cartographer 纯定位提供（无 AMCL，无 odom frame）"
echo ""
echo "操作步骤："
echo "  1. Cartographer 自动从 pbstream 恢复位姿，无需手动设置 2D Pose Estimate"
echo "  2. 如果位姿偏移，在 RViz 中点击 '2D Pose Estimate' 修正初始位姿"
echo "  3. 点击 '2D Goal Pose'，在地图上设定目标点"
echo "  4. 机器人自主规划路径并导航"
echo ""
echo "按 Ctrl+C 停止所有节点"
echo "================================================="

function shutdown() {
    echo ""
    echo "[关闭] 停止所有节点..."
    kill $GAZEBOPID $BASEPID $SLAMPID $SCANPID $NAVPID $RVIZPID 2>/dev/null
    cleanup
    echo "[完成]"
    exit 0
}
trap shutdown SIGINT

wait
