#!/bin/bash
# 保存 Cartographer 地图为 Nav2 格式（PGM + YAML）
#
# 用法:
#   ./save_map.sh [地图名]
#
# 示例:
#   ./save_map.sh g60pro     -> 自动保存为 g60pro_v1, g60pro_v2 ...
#   ./save_map.sh mymap      -> 自动保存为 mymap_v1, mymap_v2 ...
#
# 注意: 需要先 source 工作空间的 setup.bash
#   source /opt/ros/humble/setup.bash
#   source install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
INSTALL_DIR="$WORKSPACE_DIR/install"
PKG_SITEPKG="$INSTALL_DIR/robot_slam/lib/python3.10/site-packages"
PKG_LIB="$INSTALL_DIR/robot_slam/lib/robot_slam"

SAVE_MAP_SCRIPT="$PKG_SITEPKG/robot_slam/save_map.py"

if [ ! -f "$SAVE_MAP_SCRIPT" ]; then
    echo "错误: 找不到 save_map.py ($SAVE_MAP_SCRIPT)"
    echo "请先编译 robot_slam 包: colcon build --packages-select robot_slam"
    exit 1
fi

export PYTHONPATH="$PKG_SITEPKG:$PYTHONPATH"
export AMENT_PREFIX_PATH="$INSTALL_DIR/robot_slam:$AMENT_PREFIX_PATH"

MAP_NAME="${1:-g60pro}"

# If first arg looks like a flag (--something or -something), use default name
if [[ "$MAP_NAME" == --* ]] || [[ "$MAP_NAME" == -* ]]; then
    # It's a flag; prepend default map name
    set -- "g60pro" "$@"
else
    # It's a map name; shift it off so remaining args pass through
    shift
fi

MAP_PATH="$WORKSPACE_DIR/maps/$MAP_NAME"

echo "正在保存地图到: $MAP_PATH"

python3 "$SAVE_MAP_SCRIPT" "$MAP_PATH" "$@"
