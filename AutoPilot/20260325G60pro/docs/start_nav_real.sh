#!/bin/bash
# G60Pro 实车 Nav2 导航启动脚本（使用 AMCL 定位）
#
# 前提：
#   1. 已有建好的地图（maps/g60pro_v*.yaml）
#   2. 当前没有 Cartographer 建图在运行（会与 AMCL 冲突）
#
# 流程:
#   Phase 1: 启动 Helios16 驱动（发布点云）
#   Phase 2: 启动 Nav2 导航栈（AMCL 定位 + 路径规划）
#   Phase 3: 用户在 RViz 初始化机器人位置
#   Phase 4: 发送导航目标
#
# 使用方式:
#   ./start_nav_real.sh                    # 使用默认地图 g60pro_v5.yaml
#   ./start_nav_real.sh g60pro_v3          # 指定其他版本地图

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/.."
MAP_NAME="${1:-g60pro_v5}"
MAP_YAML="${WS_DIR}/maps/${MAP_NAME}.yaml"

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
    pkill -9 -f amcl                  2>/dev/null || true
    pkill -9 -f robot_base_node      2>/dev/null || true
    pkill -9 -f can_node             2>/dev/null || true
    # 清除残留的 map→base_footprint 静态 TF（会干扰 AMCL）
    pkill -9 -f "static_transform_publisher.*map.*base_footprint" 2>/dev/null || true
    sleep 1
}

# ========== Phase 0: 检查 ==========
echo "=========================================="
echo "G60Pro 实车 Nav2 导航（AMCL 定位）"
echo "=========================================="
echo ""

if [ ! -f "$MAP_YAML" ]; then
    echo "[错误] 地图文件不存在: $MAP_YAML"
    echo ""
    echo "可用地图："
    ls -1 "${WS_DIR}"/maps/g60pro_v*.yaml 2>/dev/null || echo "  无"
    echo ""
    echo "用法: $0 [地图名]"
    echo "  $0              # 使用 g60pro_v5"
    echo "  $0 g60pro_v3    # 使用 g60pro_v3"
    exit 1
fi

echo "[检查] 使用地图: $MAP_YAML"

# 检查是否有 Cartographer 在运行
if pgrep -f cartographer_node > /dev/null 2>&1; then
    echo "[警告] 检测到 Cartographer 正在运行！"
    echo "Cartographer 建图模式和 AMCL 不能同时运行（TF 冲突）。"
    echo "请先关闭 Cartographer，再运行本脚本。"
    exit 1
fi

# ========== Phase 1: 初始化 CAN 总线 ==========
echo ""
echo "[Phase 1/4] 初始化 CAN 总线..."
# can0 = IPC CAN 接口，连接 RDM 的 CAN1（500kbps）
# CAN4 是 RDM 内部通道，与 IPC 无关
sudo ip link set can0 up type can bitrate 500000 2>/dev/null || echo "  [警告] can0 初始化失败，跳过"

# ========== Phase 2: source 环境 ==========
echo "[Phase 2/4] 加载 ROS2 环境..."
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# ========== Phase 3: 清理残留 ==========
echo "[Phase 3/4] 清理残留进程..."
cleanup_procs

# ========== Phase 4: 启动机器人描述（TF） ==========
echo "[Phase 4/4] 启动机器人描述节点..."
ros2 launch robot_description description.launch.py &
DESCRIPTION_PID=$!
sleep 3
echo "  robot_description PID: $DESCRIPTION_PID"

# ========== Phase 5: 启动 Helios16 驱动 ==========
echo ""
echo "[启动] Helios16 激光雷达驱动..."
ros2 run rslidar_sdk rslidar_sdk_node \
  --ros-args \
  -p config_path:="$WS_DIR/src/rslidar_sdk/config/config.yaml" &
LIDAR_PID=$!
echo "  rslidar_sdk PID: $LIDAR_PID"
sleep 3

# ========== Phase 6: 启动 Nav2 导航 ==========
echo ""
echo "[启动] Nav2 导航栈（AMCL 定位 + 路径规划）..."
echo "  地图: $MAP_YAML"
echo "  AMCL: 使用已保存地图进行激光定位"
echo "  pointcloud_to_laserscan: /lidar/rs16/points -> /scan"
echo ""
echo "  启动内容："
echo "    - map_server（加载 $MAP_NAME）"
echo "    - amcl（激光定位，订阅 /scan）"
echo "    - planner_server（全局路径规划）"
echo "    - controller_server（局部路径规划/DWB）"
echo "    - global_costmap + local_costmap（避障）"
echo "    - velocity_smoother（速度平滑）"
echo "    - behavior_server（旋转/后退/等待）"
echo ""

ros2 launch robot_navigation navigation_real.launch.py map:="$MAP_YAML" &
NAV_PID=$!
echo "  Nav2 导航 PID: $NAV_PID"
sleep 5

# ========== Phase 7: 启动底盘节点（里程计） ==========
echo ""
echo "[启动] robot_base_node（底盘运动学 + /odom + odom→base_footprint TF）..."
ros2 run robot_base robot_base_node \
  --ros-args \
  -p use_sim:=false \
  -p use_sim_time:=false &
BASE_PID=$!
echo "  robot_base_node PID: $BASE_PID"
sleep 2

# ========== Phase 8: 启动 CAN 节点（底盘 CAN 通信） ==========
echo ""
echo "[启动] can_node（IPC ↔ RDM CAN 通信）..."
ros2 run robot_can can_node &
CAN_PID=$!
echo "  can_node PID: $CAN_PID"
sleep 2

# ========== Phase 9: 启动 RViz ==========
echo "[启动] RViz（导航视图）..."
# 如果有专门的导航 RViz 配置就用它，否则用 slam_real 配置（显示地图和激光）
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
echo "  - Nav2 导航:         $NAV_PID"
echo "  - robot_base_node:   $BASE_PID"
echo "  - can_node:          $CAN_PID"
echo "  - RViz:              $RVIZ_PID"
echo ""
echo "使用地图: $MAP_NAME"
echo ""
echo "=========================================="
echo "RViz 操作步骤："
echo ""
echo "  1. 【重要】点击 '2D Pose Estimate' 按钮"
echo "     → 在地图上点击机器人实际所在位置"
echo "     → 拖拽设置机器人朝向（必须准确！）"
echo "     → 这告诉 AMCL 机器人初始位置在哪里"
echo ""
echo "  2. 点击 '2D Nav Goal' 按钮"
echo "     → 在地图上点击目标位置"
echo "     → 拖拽设置目标朝向"
echo "     → 机器人将自主导航到目标点"
echo ""
echo "=========================================="
echo "话题说明："
echo "  /map              <- 静态地图（map_server 发布）"
echo "  /scan             <- 2D 激光（pointcloud_to_laserscan 转换）"
echo "  /odom              <- 里程计（robot_base_node 开环积分，cmd_vel驱动）"
echo "  /plan              <- 全局规划路径"
echo "  /local_plan        <- 局部规划路径"
echo "  /cmd_vel          -> 速度指令（-> 底盘 CAN）"
echo ""
echo "如果 AMCL 定位飘："
echo "  → 多点击几次 '2D Pose Estimate' 重新初始化"
echo "  → 检查 /scan 话题是否有数据（pointcloud_to_laserscan）"
echo ""
echo "按 Ctrl+C 停止所有节点"
echo "=========================================="

# ========== 清理函数 ==========
function shutdown() {
    echo ""
    echo "[关闭] 停止所有节点..."
    kill $DESCRIPTION_PID $LIDAR_PID $NAV_PID $BASE_PID $CAN_PID $RVIZ_PID 2>/dev/null
    cleanup_procs
    echo "[完成]"
    exit 0
}
trap shutdown SIGINT

wait
