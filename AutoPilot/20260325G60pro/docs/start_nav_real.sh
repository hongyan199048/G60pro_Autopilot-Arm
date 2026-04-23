#!/bin/bash
# G60Pro 实车 Nav2 导航启动脚本（Cartographer 纯定位模式）
#
# 前提：
#   1. 已有建好的地图（maps/g60pro_v*.pbstream + .yaml）
#   2. 当前没有 Cartographer 建图在运行
#
# 流程:
#   Phase 1: 初始化 CAN 总线
#   Phase 2: 启动 Helios16 驱动（发布点云）
#   Phase 3: 启动 Cartographer 纯定位（加载 .pbstream 地图）
#   Phase 4: 启动 Nav2 导航栈（路径规划）
#   Phase 5: 启动底盘 + CAN 节点
#   Phase 6: 启动 RViz
#   Phase 7: 用户在 RViz 确认定位（2D Pose Estimate）
#   Phase 8: 发送导航目标
#
# 使用方式:
#   ./start_nav_real.sh                    # 使用 g60pro_v5.pbstream（纯定位）
#   ./start_nav_real.sh g60pro_v6          # 指定其他版本地图（纯定位）
#   ./start_nav_real.sh --yaml g60pro_v6  # 使用 .yaml 静态地图（实时 /map）

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/.."
USE_YAML=false

# 解析参数
if [ "$1" == "--yaml" ]; then
    USE_YAML=true
    MAP_NAME="${2:-g60pro_v5}"
elif [ -n "$1" ]; then
    MAP_NAME="$1"
else
    MAP_NAME="g60pro_v5"
fi

MAP_YAML="${WS_DIR}/maps/${MAP_NAME}.yaml"
MAP_PBSTREAM="${WS_DIR}/maps/${MAP_NAME}.pbstream"

# ========== 辅助函数 ==========
function pause() {
    echo ""
    read -p "按 Enter 继续..." key
}

function cleanup_procs() {
    echo "[清理] 停止残留进程..."
    pkill -9 -f cartographer_node    2>/dev/null || true
    pkill -9 -f rslidar_sdk_node     2>/dev/null || true
    pkill -9 -f robot_state_publisher 2>/dev/null || true
    pkill -9 -f rviz2                2>/dev/null || true
    pkill -9 -f map_server           2>/dev/null || true
    pkill -9 -f robot_base_node      2>/dev/null || true
    pkill -9 -f can_node             2>/dev/null || true
    sleep 1
}

# ========== Phase 0: 检查 ==========
echo "=========================================="
echo "G60Pro 实车 Nav2 导航（纯定位模式）"
echo "=========================================="
echo ""

if $USE_YAML; then
    # YAML 模式：检查 .yaml 文件
    if [ ! -f "$MAP_YAML" ]; then
        echo "[错误] 地图文件不存在: $MAP_YAML"
        echo ""
        echo "可用地图："
        ls -1 "${WS_DIR}"/maps/g60pro_v*.yaml 2>/dev/null || echo "  无"
        echo ""
        exit 1
    fi
    echo "[检查] 使用 .yaml 静态地图: $MAP_YAML"
else
    # 纯定位模式：检查 .pbstream 文件
    if [ ! -f "$MAP_PBSTREAM" ]; then
        echo "[错误] .pbstream 文件不存在: $MAP_PBSTREAM"
        echo ""
        echo "可用地图："
        ls -1 "${WS_DIR}"/maps/g60pro_v*.pbstream 2>/dev/null || echo "  无"
        echo ""
        echo "用法: $0 --yaml $MAP_NAME  # 改用 .yaml 静态地图模式"
        exit 1
    fi
    echo "[检查] 使用 .pbstream 纯定位地图: $MAP_PBSTREAM"
fi

# 检查是否有 Cartographer 在运行
if pgrep -f cartographer_node > /dev/null 2>&1; then
    echo "[警告] 检测到 Cartographer 正在运行！"
    echo "请先关闭 Cartographer，再运行本脚本。"
    exit 1
fi

# ========== Phase 1: 初始化 CAN 总线 ==========
echo ""
echo "[Phase 1/8] 初始化 CAN 总线..."
sudo ip link set can0 up type can bitrate 500000 2>/dev/null || echo "  [警告] can0 初始化失败，跳过"

# ========== Phase 2: source 环境 ==========
echo "[Phase 2/8] 加载 ROS2 环境..."
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# ========== Phase 3: 清理残留 ==========
echo "[Phase 3/8] 清理残留进程..."
cleanup_procs

# ========== Phase 4: 启动机器人描述（TF） ==========
echo "[Phase 4/8] 启动机器人描述节点..."
ros2 launch robot_description description.launch.py &
DESCRIPTION_PID=$!
sleep 3
echo "  robot_description PID: $DESCRIPTION_PID"

# ========== Phase 5: 启动 Helios16 驱动 ==========
echo ""
echo "[Phase 5/8] 启动 Helios16 激光雷达驱动..."
ros2 run rslidar_sdk rslidar_sdk_node \
  --ros-args \
  -p config_path:="$WS_DIR/src/rslidar_sdk/config/config.yaml" &
LIDAR_PID=$!
echo "  rslidar_sdk PID: $LIDAR_PID"
sleep 3

# ========== Phase 6: 启动 Cartographer 纯定位 ==========
# slam_real_localization.launch.py 加载 .pbstream 文件，
# 发布 map→base_footprint（来自 .pbstream 地图坐标系的原始坐标）
# 不构建新地图，纯做 scan matching 定位
echo ""
if $USE_YAML; then
    echo "[Phase 6/8] 启动 Cartographer 纯定位（加载 .pbstream 地图）..."
    echo "  地图: $MAP_PBSTREAM"
    echo "  可视化: $MAP_YAML（map_server 提供 /map）"
    # 纯定位模式：加载 .pbstream 做 scan matching，同时 Nav2 用 .yaml 显示静态地图
    ros2 launch robot_slam slam_real_localization.launch.py \
      use_sim_time:=false \
      pbstream_file:="$MAP_PBSTREAM" &
    CARTO_PID=$!
    sleep 4
else
    echo "[Phase 6/8] 启动 Cartographer 纯定位（加载 .pbstream 地图）..."
    echo "  地图: $MAP_PBSTREAM"
    # 纯定位模式：加载 cartographer_real_localization.lua + .pbstream
    ros2 launch robot_slam slam_real_localization.launch.py \
      use_sim_time:=false \
      pbstream_file:="$MAP_PBSTREAM" &
    CARTO_PID=$!
    sleep 4
fi
echo "  Cartographer PID: $CARTO_PID"

# ========== Phase 7: 启动 Nav2 导航（无 AMCL） ==========
echo ""
echo "[Phase 7/8] 启动 Nav2 导航栈..."
echo "  定位: Cartographer scan matching（map→base_footprint）"
echo "  pointcloud_to_laserscan: /lidar/rs16/points -> /scan"
echo ""
echo "  启动内容："
echo "    - planner_server（全局路径规划）"
echo "    - controller_server（局部路径规划/DWB）"
echo "    - global_costmap + local_costmap（避障）"
echo "    - velocity_smoother（速度平滑）"
echo "    - behavior_server（旋转/后退/等待）"
echo ""

if $USE_YAML; then
    # YAML 模式：只传 .yaml 路径，不传 pbstream_file（默认为空）
    ros2 launch robot_navigation navigation_real.launch.py \
      map:="$MAP_YAML" &
else
    # 纯定位模式：只传 .pbstream 路径，不传 map（默认为空）
    ros2 launch robot_navigation navigation_real.launch.py \
      pbstream_file:="$MAP_PBSTREAM" &
fi
NAV_PID=$!
echo "  Nav2 导航 PID: $NAV_PID"
sleep 5

# ========== Phase 8: 启动底盘节点（里程计） ==========
echo ""
echo "[启动] robot_base_node（底盘运动学 + /odom + odom→base_footprint TF）..."
ros2 run robot_base robot_base_node \
  --ros-args \
  -p use_sim:=false \
  -p use_sim_time:=false \
  -p publish_tf:=false &
BASE_PID=$!
echo "  robot_base_node PID: $BASE_PID"
sleep 2

# ========== 启动 CAN 节点（底盘 CAN 通信） ==========
echo ""
echo "[启动] can_node（IPC ↔ RDM CAN 通信）..."
ros2 run robot_can can_node &
CAN_PID=$!
echo "  can_node PID: $CAN_PID"
sleep 2

# ========== 启动 RViz ==========
echo "[启动] RViz（导航视图）..."
if [ -f "${WS_DIR}/src/robot_rviz/rviz/navigation_real.rviz" ]; then
    RVIZ_CONFIG="${WS_DIR}/src/robot_rviz/rviz/navigation_real.rviz"
elif [ -f "${WS_DIR}/src/robot_rviz/rviz/slam_real.rviz" ]; then
    RVIZ_CONFIG="${WS_DIR}/src/robot_rviz/rviz/slam_real.rviz"
else
    RVIZ_CONFIG=""
fi

if [ -n "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG" &
else
    rviz2 &
fi
RVIZ_PID=$!
echo "  RViz PID: $RVIZ_PID"

sleep 2

# ========== 完成 ==========
echo ""
echo "=========================================="
echo "  Nav2 导航已启动！"
echo "=========================================="
echo ""
echo "当前运行的进程："
echo "  - robot_description: $DESCRIPTION_PID"
echo "  - rslidar_sdk:       $LIDAR_PID"
echo "  - Cartographer:      $CARTO_PID"
echo "  - Nav2 导航:         $NAV_PID"
echo "  - robot_base_node:   $BASE_PID"
echo "  - can_node:          $CAN_PID"
echo "  - RViz:              $RVIZ_PID"
echo ""

if $USE_YAML; then
    echo "使用地图: $MAP_YAML（实时建图模式）"
else
    echo "使用地图: $MAP_PBSTREAM（纯定位模式）"
fi

echo ""
echo "=========================================="
echo "操作步骤："
echo ""
echo "  1. 在 RViz 中点击 '2D Pose Estimate' 按钮"
echo "     → 在地图上点击机器人当前位置"
echo "     → 拖拽设置机器人朝向（纯定位模式必须！）"
echo ""
echo "  2. 点击 '2D Nav Goal' 按钮"
echo "     → 在地图上点击目标位置"
echo "     → 拖拽设置目标朝向"
echo "     → 机器人将自主导航到目标点"
echo ""
echo "=========================================="
echo "话题说明："
echo "  /map              <- 地图（YAML 模式：map_server；纯定位模式：无）"
echo "  /scan             <- 2D 激光（pointcloud_to_laserscan 转换）"
echo "  /tf               <- map→base_footprint（Cartographer 发布）"
echo "  /odom             <- 里程计（robot_base_node 开环积分）"
echo ""
echo "按 Ctrl+C 停止所有节点"
echo "=========================================="

# ========== 清理函数 ==========
function shutdown() {
    echo ""
    echo "[关闭] 停止所有节点..."
    kill $DESCRIPTION_PID $LIDAR_PID $CARTO_PID $NAV_PID $BASE_PID $CAN_PID $RVIZ_PID 2>/dev/null
    cleanup_procs
    echo "[完成]"
    exit 0
}
trap shutdown SIGINT

wait