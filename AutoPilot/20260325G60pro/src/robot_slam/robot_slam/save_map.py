#!/usr/bin/env python3
"""
从 /map 话题保存 Cartographer 占据网格为 PGM + YAML (Nav2 格式)

用法:
  ros2 run robot_slam save_map <保存路径(不含后缀)>

示例:
  ros2 run robot_slam save_map /path/to/maps/g60pro
    -> 自动检测已有版本，保存为 g60pro_v1、g60pro_v2 ...

  ros2 run robot_slam save_map /path/to/maps/g60pro_v3
    -> 保存为 g60pro_v3（覆盖模式，文件名已有 _vN 则用该版本号）

  ros2 run robot_slam save_map -f /path/to/maps/g60pro
    -> 强制保存为 g60pro（覆盖同名文件）
"""

import sys
import os
import re
import argparse
import numpy as np
from PIL import Image
import yaml

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


def find_next_version(base_path):
    """
    在 base_path 目录下查找 base_path_v{1,2,...} 形式的已有文件，
    返回下一个版本号对应的路径。
    例如 base_path = '/maps/g60pro'：
      已有 g60pro_v1.pgm -> 返回 '/maps/g60pro_v2'
      已有 g60pro_v1.pgm, g60pro_v3.pgm -> 返回 '/maps/g60pro_v4'
      没有找到 -> 返回 '/maps/g60pro_v1'
    """
    directory = os.path.dirname(base_path)
    basename = os.path.basename(base_path)
    pattern = re.compile(rf'^{re.escape(basename)}_v(\d+)\.pgm$')

    max_version = 0
    if os.path.exists(directory):
        for fname in os.listdir(directory):
            m = pattern.match(fname)
            if m:
                max_version = max(max_version, int(m.group(1)))

    next_version = max_version + 1
    return f"{base_path}_v{next_version}"


class MapSaver(Node):
    def __init__(self, save_path):
        super().__init__('map_saver')
        self.save_path = save_path
        self.map_data = None
        self.map_info = None

        self.get_logger().info(f'等待 /map 话题... (目标: {save_path})')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            1
        )

        # 最多等待 30 秒获取地图
        timeout = 30.0
        start = self.get_clock().now()
        while rclpy.ok() and self.map_data is None:
            rclpy.spin_once(self, timeout_sec=0.5)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().error('超时：未收到地图数据')
                sys.exit(1)

        self.save_map()

    def map_callback(self, msg):
        self.map_data = msg.data
        self.map_info = msg.info
        self.get_logger().info(
            f'收到地图: {msg.info.width}x{msg.info.height}, '
            f'分辨率: {msg.info.resolution:.3f}m'
        )

    def save_map(self):
        w = self.map_info.width
        h = self.map_info.height
        data = np.array(self.map_data, dtype=np.int8).reshape((h, w))

        # occupancy: 0=free(254), -1=unknown(205), 100=occupied(0)
        img = np.full((h, w), 205, dtype=np.uint8)
        img[data == 0] = 254    # free
        img[data == 100] = 0    # occupied
        img[data == -1] = 205   # unknown
        img = np.flipud(img)    # 图像 y 轴朝上

        # 保存 PGM
        pgm_path = self.save_path + '.pgm'
        Image.fromarray(img).save(pgm_path)
        self.get_logger().info(f'PGM 已保存: {pgm_path}')

        # 保存 YAML (Nav2 格式)
        yaml_path = self.save_path + '.yaml'
        yaml_data = {
            'image': os.path.basename(pgm_path),
            'mode': 'trinary',
            'resolution': self.map_info.resolution,
            'origin': [
                self.map_info.origin.position.x,
                self.map_info.origin.position.y,
                self.map_info.origin.orientation.z
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)
        self.get_logger().info(f'YAML 已保存: {yaml_path}')
        self.get_logger().info(f'地图保存完成: {self.save_path}')


def main():
    parser = argparse.ArgumentParser(
        description='保存 Cartographer /map 话题为 Nav2 可用的 PGM + YAML 文件',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        'save_path',
        nargs='?',
        help='保存路径（不含后缀）。例如 /path/to/maps/g60pro'
    )
    parser.add_argument(
        '-f', '--force',
        action='store_true',
        help='强制使用指定路径（覆盖已有文件），不自动加版本号'
    )
    args = parser.parse_args(sys.argv[1:])

    if not args.save_path:
        parser.print_help()
        sys.exit(1)

    save_path = args.save_path

    if not args.force:
        # 检查是否已有同名文件，如有则自动加版本号
        if os.path.exists(save_path + '.pgm') or os.path.exists(save_path + '.yaml'):
            print(f'⚠ 文件已存在，自动加版本号...')
            save_path = find_next_version(save_path)
        else:
            # 检查是否有同系列的版本文件
            test_path = find_next_version(save_path)
            # find_next_version 总是返回 _v{next}，所以用原路径即可
            pass

    rclpy.init(args=sys.argv)
    node = MapSaver(save_path)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
