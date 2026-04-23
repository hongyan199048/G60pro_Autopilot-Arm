#!/bin/bash
# G60Pro 导航系统监控脚本
# 用途：实时监控定位、匹配、导航状态

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/.."

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
function print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

function print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

function print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

function print_ok() {
    echo -e "${GREEN}[OK]${NC} $1"
}

# ========== 监控项 ==========

# 1. Cartographer 匹配得分监控（实时）
function monitor_cartographer_score() {
    echo ""
    print_info "=== Cartographer 匹配得分实时监控 ==="
    print_info "实时显示匹配日志（score > 70% 为良好，Ctrl+C 退出）"
    echo ""

    # 从最新的 Cartographer 日志中提取匹配得分
    LOG_FILE=$(find ~/.ros/log -type f -name "*cartographer*.log" | xargs ls -t 2>/dev/null | head -1)
    if [ -z "$LOG_FILE" ]; then
        print_warn "未找到 Cartographer 日志"
        return
    fi

    if [ ! -f "$LOG_FILE" ]; then
        print_warn "日志文件不存在: $LOG_FILE"
        return
    fi

    print_info "监控日志: $LOG_FILE"
    echo ""

    # 实时 tail 日志并过滤匹配得分
    tail -f "$LOG_FILE" | grep --line-buffered "matches with score" | while read line; do
        score=$(echo "$line" | grep -oP 'score \K[0-9.]+')
        timestamp=$(date '+%H:%M:%S')
        if (( $(echo "$score > 70" | bc -l) )); then
            print_ok "[$timestamp] $line"
        elif (( $(echo "$score > 50" | bc -l) )); then
            print_warn "[$timestamp] $line"
        else
            print_error "[$timestamp] $line"
        fi
    done
}

# 2. TF 稳定性监控
function monitor_tf_stability() {
    echo ""
    print_info "=== TF 稳定性监控（map→base_footprint）==="
    print_info "采样 5 次，检查位置是否稳定"
    echo ""

    for i in {1..5}; do
        result=$(ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | grep -A3 "Translation" | head -4)
        if [ -n "$result" ]; then
            echo "$result"
            echo "---"
        else
            print_error "无法获取 TF"
        fi
        sleep 0.5
    done
}

# 3. 节点状态监控
function monitor_nodes() {
    echo ""
    print_info "=== 关键节点状态 ==="

    nodes=(
        "cartographer"
        "map_server"
        "controller_server"
        "planner_server"
        "lifecycle_manager_navigation"
        "robot_base_node"
        "can_node"
    )

    for node in "${nodes[@]}"; do
        if ros2 node list 2>/dev/null | grep -q "/$node"; then
            print_ok "$node 运行中"
        else
            print_error "$node 未运行"
        fi
    done
}

# 4. 话题发布频率监控
function monitor_topics() {
    echo ""
    print_info "=== 关键话题发布频率 ==="

    topics=(
        "/map:期望=静态或低频"
        "/scan:期望=10Hz"
        "/odom:期望=50Hz"
        "/cmd_vel:期望=按需"
        "/lidar/rs16/points:期望=10Hz"
    )

    for topic_info in "${topics[@]}"; do
        topic=$(echo "$topic_info" | cut -d: -f1)
        expect=$(echo "$topic_info" | cut -d: -f2)

        if ros2 topic list 2>/dev/null | grep -q "^$topic$"; then
            hz=$(timeout 3 ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | awk '{print $3}')
            if [ -n "$hz" ]; then
                print_ok "$topic: ${hz} Hz ($expect)"
            else
                print_warn "$topic: 无数据"
            fi
        else
            print_error "$topic: 话题不存在"
        fi
    done
}

# 5. Lifecycle 节点状态监控
function monitor_lifecycle() {
    echo ""
    print_info "=== Lifecycle 节点状态 ==="

    lifecycle_nodes=(
        "map_server"
        "controller_server"
        "planner_server"
    )

    for node in "${lifecycle_nodes[@]}"; do
        state=$(ros2 lifecycle get "/$node" 2>/dev/null)
        if [ $? -eq 0 ]; then
            if echo "$state" | grep -q "active"; then
                print_ok "$node: $state"
            else
                print_warn "$node: $state"
            fi
        else
            print_error "$node: 无法获取状态"
        fi
    done
}

# 6. CAN 数据监控（实时原始报文解析）
function monitor_can_data() {
    echo ""
    print_info "=== CAN 数据监控（直连 can0 原始报文，Ctrl+C 退出）==="
    print_info "监控 ID: 0x019 转速反馈 | 0x01B 转角反馈 | 0x210 转速指令 | 0x211 转向角指令"
    echo ""

    python3 -c "
import os, sys, time, can, cantools.database as cd

DBC_PATH = '/home/admin123/Development/G60Pro/AutoPilot/20260325G60pro/src/robot_can/config/can1/G60_CAN1_RDM_V1.0.dbc'
CAN_CHANNEL = 'can0'
MONITOR_IDS = [0x019, 0x01B, 0x210, 0x211]
ID_NAMES = {
    0x019: 'RDM_Fr35 (0x019) 四轮转速反馈',
    0x01B: 'RDM_Fr36 (0x01B) 四轮转向角反馈',
    0x210: 'LAS_Fr01 (0x210) 转速指令',
    0x211: 'LAS_Fr02 (0x211) 转向角指令',
}
NC = '\033[0m'
RED = '\033[0;31m'
GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
BLUE = '\033[0;34m'
CYAN = '\033[0;36m'

if not os.path.exists(DBC_PATH):
    print(f'{RED}DBC 文件不存在: {DBC_PATH}{NC}')
    sys.exit(1)

db = cd.load_file(DBC_PATH)
msg_defs = {}
for cid in MONITOR_IDS:
    try:
        msg_defs[cid] = db.get_message_by_frame_id(cid)
    except Exception:
        pass

can_filters = [{'can_id': cid, 'can_mask': 0xFFF} for cid in MONITOR_IDS]
try:
    bus = can.interface.Bus(channel=CAN_CHANNEL, bustype='socketcan', can_filters=can_filters)
except Exception as e:
    print(f'{RED}无法连接 {CAN_CHANNEL}: {e}{NC}')
    sys.exit(1)

cache = {}
last_refresh = 0
loop_count = 0

try:
    while True:
        msg = bus.recv(timeout=0.05)
        now = time.time()
        if msg and msg.arbitration_id in MONITOR_IDS:
            cache[msg.arbitration_id] = {'data': bytes(msg.data), 'time': now}
        if now - last_refresh >= 0.2:
            last_refresh = now
            loop_count += 1
            os.system('clear')
            print(f'{GREEN}=== CAN 实时监控（can0 原始报文解析）==={NC}')
            print(f'{GREEN}刷新 #{loop_count}  |  {time.strftime(\"%H:%M:%S\")}  |  按 Ctrl+C 退出{NC}')
            print()
            for cid in MONITOR_IDS:
                is_rx = 'Rx' if cid <= 0x1FF else 'Tx'
                color = CYAN if is_rx == 'Tx' else GREEN
                print(f'{color}[{is_rx}] {ID_NAMES[cid]}{NC}')
                if cid in cache:
                    age = now - cache[cid]['time']
                    if age < 1.0:
                        print(f'  最近报文: {age*1000:.0f}ms 前')
                        if cid in msg_defs:
                            try:
                                decoded = msg_defs[cid].decode(cache[cid]['data'])
                                for sig in msg_defs[cid].signals:
                                    val = decoded.get(sig.name, 0)
                                    unit = sig.unit if sig.unit else ''
                                    print(f'  {sig.name:<36s} {val:>10.2f} {unit}')
                            except Exception as e:
                                print(f'  {RED}解析错误: {e}{NC}')
                                raw = cache[cid]['data'].hex(' ').upper()
                                print(f'  原始数据: {raw}')
                        else:
                            raw = cache[cid]['data'].hex(' ').upper()
                            print(f'  原始数据: {raw}  (无 DBC 定义)')
                    else:
                        print(f'  {YELLOW}[N/A] 超过 1 秒未收到报文 (上次 {age:.1f}s 前){NC}')
                else:
                    print(f'  {YELLOW}[N/A] 从未收到报文{NC}')
                print()
except KeyboardInterrupt:
    pass
finally:
    bus.shutdown()
    print('\n已退出监控')
"
}

# 7. 轮子转角监控（指令 vs 反馈）
function monitor_wheel_angles() {
    echo ""
    print_info "=== 轮子转角监控（实时，Ctrl+C 退出）==="
    print_info "FL=左前, FR=右前, RL=左后, RR=右后"
    echo ""

    # 检查话题是否存在
    if ! ros2 topic list 2>/dev/null | grep -q "^/motor_cmd$"; then
        print_error "/motor_cmd 话题不存在，请先启动导航或底盘节点"
        return
    fi

    print_info "开始监控，等待数据..."
    echo ""

    # 实时监控（每 0.5 秒刷新一次）
    local loop_count=0
    while true; do
        loop_count=$((loop_count + 1))

        # 获取指令转角
        cmd_data=$(timeout 1 ros2 topic echo /motor_cmd robot_msgs/msg/MotorCmd --once 2>/dev/null)
        cmd_status=$?

        if [ $cmd_status -eq 0 ] && [ -n "$cmd_data" ]; then
            # 解析 steer_angle 数组
            cmd_angles=$(echo "$cmd_data" | grep -A1 "steer_angle:" | tail -1 | tr -d ' []')
            IFS=',' read -ra cmd_arr <<< "$cmd_angles"
        else
            cmd_arr=("N/A" "N/A" "N/A" "N/A")
        fi

        # 获取反馈转角
        state_data=$(timeout 1 ros2 topic echo /motor_state robot_msgs/msg/MotorState --once 2>/dev/null)
        state_status=$?

        if [ $state_status -eq 0 ] && [ -n "$state_data" ]; then
            fb_angles=$(echo "$state_data" | grep -A1 "steer_angle_feedback:" | tail -1 | tr -d ' []')
            IFS=',' read -ra fb_arr <<< "$fb_angles"
        else
            fb_arr=("N/A" "N/A" "N/A" "N/A")
        fi

        # 清屏并显示
        clear
        echo -e "${BLUE}=== 轮子转角监控（实时）===${NC}"
        echo -e "${BLUE}更新次数: $loop_count${NC}"
        echo ""

        # 显示话题状态
        if [ $cmd_status -eq 124 ]; then
            echo -e "${YELLOW}⚠ /motor_cmd: 等待数据...（无控制指令）${NC}"
        elif [ $cmd_status -eq 0 ]; then
            echo -e "${GREEN}✓ /motor_cmd: 数据正常${NC}"
        else
            echo -e "${RED}✗ /motor_cmd: 读取失败${NC}"
        fi

        if [ $state_status -eq 124 ]; then
            echo -e "${YELLOW}⚠ /motor_state: 等待数据...（CAN 未连接？）${NC}"
        elif [ $state_status -eq 0 ]; then
            echo -e "${GREEN}✓ /motor_state: 数据正常${NC}"
        else
            echo -e "${RED}✗ /motor_state: 读取失败${NC}"
        fi

        echo ""
        printf "%-8s %-20s %-20s %-15s\n" "轮子" "指令 (rad)" "反馈 (rad)" "误差 (rad)"
        echo "------------------------------------------------------------"

        wheels=("FL" "FR" "RL" "RR")
        for i in {0..3}; do
            cmd_val="${cmd_arr[$i]}"
            fb_val="${fb_arr[$i]}"

            # 计算误差（如果都是数字）
            if [[ "$cmd_val" =~ ^-?[0-9.]+$ ]] && [[ "$fb_val" =~ ^-?[0-9.]+$ ]]; then
                error=$(echo "$cmd_val - $fb_val" | bc -l 2>/dev/null)
                error_abs=$(echo "$error" | sed 's/-//')

                # 根据误差大小着色
                if (( $(echo "$error_abs > 0.1" | bc -l) )); then
                    error_str="${RED}${error}${NC}"
                elif (( $(echo "$error_abs > 0.05" | bc -l) )); then
                    error_str="${YELLOW}${error}${NC}"
                else
                    error_str="${GREEN}${error}${NC}"
                fi
            else
                error_str="N/A"
            fi

            printf "%-8s %-20s %-20s %-15s\n" "${wheels[$i]}" "$cmd_val" "$fb_val" "$(echo -e $error_str)"
        done

        echo ""
        echo -e "${YELLOW}提示：误差 >0.1rad(5.7°)=红色, >0.05rad(2.9°)=黄色, ≤0.05rad=绿色${NC}"
        echo -e "${BLUE}按 Ctrl+C 退出${NC}"

        sleep 0.5
    done
}

# ========== 主菜单 ==========
function show_menu() {
    echo ""
    echo "=========================================="
    echo "  G60Pro 导航系统监控"
    echo "=========================================="
    echo ""
    echo "1) Cartographer 匹配得分"
    echo "2) TF 稳定性（map→base_footprint）"
    echo "3) 节点状态"
    echo "4) 话题发布频率"
    echo "5) Lifecycle 节点状态"
    echo "6) CAN 数据监控（实时数值）"
    echo "7) 轮子转角监控（指令 vs 反馈）"
    echo "8) 全部监控"
    echo "0) 退出"
    echo ""
    read -p "请选择监控项 [0-8]: " choice

    case $choice in
        1) monitor_cartographer_score ;;
        2) monitor_tf_stability ;;
        3) monitor_nodes ;;
        4) monitor_topics ;;
        5) monitor_lifecycle ;;
        6) monitor_can_data ;;
        7) monitor_wheel_angles ;;
        8)
            monitor_nodes
            monitor_lifecycle
            monitor_topics
            monitor_tf_stability
            monitor_cartographer_score
            ;;
        0) exit 0 ;;
        *) print_error "无效选择" ;;
    esac

    echo ""
    read -p "按 Enter 返回菜单..." key
    show_menu
}

# ========== 启动 ==========
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

if [ "$1" == "--all" ]; then
    # 非交互模式：执行全部监控
    monitor_nodes
    monitor_lifecycle
    monitor_topics
    monitor_tf_stability
    monitor_cartographer_score
else
    # 交互模式：显示菜单
    show_menu
fi
