#!/bin/bash
# ============================================================
# G60Pro 实车 SLAM 一键启动脚本
# ============================================================
# 功能：启动完整的 Cartographer SLAM 建图系统
# 适用：Helios16 多线激光雷达 + Cartographer 2D 建图（不依赖 IMU）
# 作者：G60Pro 团队
# 日期：2026-04-25
# ============================================================

# 错误时立即退出（任何命令返回非 0 则脚本终止）
set -e

# ========== 路径配置 ==========
# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# 工作空间根目录（脚本在 docs/ 下，所以上一级是工作空间）
WS_DIR="$SCRIPT_DIR/.."

echo "=========================================="
echo "G60Pro 实车 Cartographer SLAM 启动"
echo "=========================================="

# ========== 第1步：清理残留进程 ==========
# 目的：避免端口占用、话题冲突、TF 树混乱
# pkill -9：强制杀死进程（SIGKILL）
# -f：匹配完整命令行（不只是进程名）
# 2>/dev/null：忽略错误输出（进程不存在时）
# || true：即使 pkill 失败也继续执行（不触发 set -e）
echo "[1/6] 清理残留进程..."
pkill -9 -f cartographer_node    2>/dev/null || true  # SLAM 核心节点
pkill -9 -f rslidar_sdk_node     2>/dev/null || true  # 雷达驱动节点
pkill -9 -f robot_state_publisher 2>/dev/null || true  # TF 发布节点
pkill -9 -f rviz2                2>/dev/null || true  # 可视化节点
sleep 1  # 等待进程完全退出，释放资源

# ========== 第2步：初始化 CAN 总线 ==========
# 目的：为后续底盘控制做准备（SLAM 阶段不使用，但提前初始化）
# can0：500kbps，工控机 ↔ RDM 控制器通信
# can1：1Mbps，RDM ↔ 电机状态反馈
# sudo：需要 root 权限操作网络接口
# 2>/dev/null || echo：失败时不中断，只打印警告
echo "[2/6] 初始化 CAN 总线..."
sudo ip link set can0 up type can bitrate 500000  2>/dev/null || echo "  [警告] can0 初始化失败，跳过（不影响建图）"
sudo ip link set can1 up type can bitrate 1000000 2>/dev/null || echo "  [警告] can1 初始化失败，跳过"

# ========== 第3步：加载 ROS2 环境 ==========
# 目的：设置 ROS2 环境变量（AMENT_PREFIX_PATH, ROS_DISTRO 等）
# 必须先 source 系统环境，再 source 工作空间环境（覆盖顺序）
echo "[3/6] 加载 ROS2 环境..."
source /opt/ros/humble/setup.bash      # ROS2 Humble 系统环境
source "$WS_DIR/install/setup.bash"    # 工作空间环境（包含自定义包）

# ========== 第4步：启动机器人描述节点 ==========
# 功能：发布 URDF 模型 + 静态 TF 树
# TF 树：base_footprint → base_link → rs16_link → ...
# & 符号：后台运行，不阻塞脚本
# $!：获取最后一个后台进程的 PID
echo "[4/6] 启动机器人描述节点（TF）..."
ros2 launch robot_description description.launch.py &
DESCRIPTION_PID=$!  # 保存 PID，用于后续清理
sleep 3  # 等待 TF 树发布完成，避免后续节点找不到 TF

# ========== 第5步：启动 Helios16 激光雷达驱动 ==========
# 功能：读取雷达数据，发布点云到 /lidar/rs16/points
# --ros-args：传递 ROS2 参数
# -p config_path：指定雷达配置文件路径
# 配置文件关键参数：
#   - use_lidar_clock: false（使用系统时间，避免 2017 年时间戳问题）
#   - lidar_type: RSHELIOS_16P
#   - frame_id: rs16_link
echo "[5/6] 启动 Helios16 激光雷达..."
ros2 run rslidar_sdk rslidar_sdk_node \
  --ros-args \
  -p config_path:="$WS_DIR/src/rslidar_sdk/config/config.yaml" &
LIDAR_PID=$!
sleep 3  # 等待雷达初始化，开始发布点云

# ========== 第6步：启动 Cartographer SLAM ==========
# 功能：订阅点云，发布地图和 TF（map → base_footprint）
# slam_real.launch.py 内部启动两个节点：
#   1. cartographer_node：SLAM 核心，加载 cartographer_real.lua
#   2. cartographer_occupancy_grid_node：将子图转为栅格地图 /map
# 配置文件：src/robot_slam/config/cartographer_real.lua
#   - tracking_frame: base_link
#   - published_frame: base_footprint
#   - use_odometry: false（不使用轮速计，纯激光 SLAM）
echo "[6/6] 启动 Cartographer SLAM..."
ros2 launch robot_slam slam_real.launch.py &
CARTO_PID=$!
GRID_PID=$CARTO_PID   # launch 内统一管理，PID 相同
sleep 4  # 等待 Cartographer 初始化，开始建图

# ========== 启动 RViz 可视化 ==========
# 功能：可视化点云、地图、TF 树、机器人模型
# -d：加载预设配置文件（Fixed Frame、显示项等）
echo "启动 RViz..."
rviz2 -d "$WS_DIR/src/robot_rviz/rviz/slam_real.rviz" &
RVIZ_PID=$!

# ========== 启动完成提示 ==========
echo ""
echo "=========================================="
echo "所有节点已启动！"
echo ""
echo "进程 PID:"
echo "  - robot_description: $DESCRIPTION_PID  (TF 静态树发布)"
echo "  - rslidar_sdk:       $LIDAR_PID        (Helios16 点云发布)"
echo "  - Cartographer:      $CARTO_PID        (SLAM 核心)"
echo "  - OccupancyGrid:     $GRID_PID         (栅格地图转换)"
echo "  - RViz:              $RVIZ_PID         (可视化)"
echo ""
echo "话题说明："
echo "  /lidar/rs16/points  -> Helios16 点云（rslidar_sdk 发布）"
echo "  /map                -> Cartographer 建图结果（OccupancyGrid）"
echo "  /tf                 -> TF 树（map → base_footprint → base_link → rs16_link）"
echo ""
echo "键盘控制（新终端）："
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/cmd_vel"
echo "  提示：使用 i/j/k/l 控制前后左右，u/o 控制旋转"
echo ""
echo "保存地图："
echo "  cd $SCRIPT_DIR && ./save_map.sh"
echo "  地图保存到：maps/g60pro_v*.pgm + .yaml"
echo ""
echo "监控话题："
echo "  ros2 topic hz /lidar/rs16/points  # 查看点云频率（应该 ~10Hz）"
echo "  ros2 topic hz /map                # 查看地图更新频率（应该 ~1Hz）"
echo ""
echo "查看 TF 树："
echo "  ros2 run tf2_tools view_frames    # 生成 frames_*.pdf"
echo ""
echo "按 Ctrl+C 停止所有节点"
echo "=========================================="

# ========== 信号捕获与清理 ==========
# trap：捕获 SIGINT（Ctrl+C）和 SIGTERM（kill）信号
# 作用：用户按 Ctrl+C 时，自动清理所有后台进程，避免残留
# kill：发送 SIGTERM 信号（优雅退出）
# 2>/dev/null：忽略进程已退出的错误
trap "echo '正在停止所有节点...'; kill $DESCRIPTION_PID $LIDAR_PID $CARTO_PID $GRID_PID $RVIZ_PID 2>/dev/null; exit 0" SIGINT SIGTERM

# ========== 等待所有后台进程 ==========
# wait：阻塞脚本，直到所有后台进程退出
# 作用：保持脚本运行，否则脚本会立即退出，后台进程变成孤儿进程
wait

# ============================================================
# 故障排查指南
# ============================================================
# 1. RViz 中机器人飘到极远处：
#    - 原因：雷达时间戳错误（2017 年）
#    - 检查：ros2 topic echo /lidar/rs16/points --once | grep stamp
#    - 修复：确认 config.yaml 中 use_lidar_clock: false
#
# 2. /map 不发布：
#    - 原因：Cartographer 崩溃或点云未收到
#    - 检查：ros2 topic hz /lidar/rs16/points
#    - 检查：ros2 node info /cartographer
#
# 3. TF 树断裂：
#    - 原因：robot_state_publisher 未启动
#    - 检查：ros2 run tf2_tools view_frames
#    - 修复：确认 description.launch.py 正常启动
#
# 4. 点云在 RViz 中不显示：
#    - 原因：Fixed Frame 设置错误
#    - 修复：RViz 左侧 Global Options → Fixed Frame 改为 rs16_link
#
# 5. 雷达无数据：
#    - 原因：网络配置错误或雷达未上电
#    - 检查：ping 192.168.2.200
#    - 检查：ifconfig enp0s31f6（应该是 192.168.2.102）
# ============================================================
