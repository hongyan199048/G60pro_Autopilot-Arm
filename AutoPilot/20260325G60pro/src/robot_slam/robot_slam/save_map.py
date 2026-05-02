#!/usr/bin/env python3
"""
保存 Cartographer 地图为 3 种格式：PGM + YAML + PBSTREAM

- PGM + YAML：Nav2 格式，用于 map_server 加载静态地图
- PBSTREAM：Cartographer 格式，用于纯定位模式加载

用法:
  ros2 run robot_slam save_map <保存路径(不含后缀)>

示例:
  ros2 run robot_slam save_map /path/to/maps/g60pro
    -> 自动检测已有版本，保存为 g60pro_v1.pgm + .yaml + .pbstream

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
from cartographer_ros_msgs.srv import WriteState


def find_next_version(base_path):
    """
    在 base_path 目录下查找 base_path_v{1,2,...} 形式的已有文件（pgm/yaml/pbstream），
    返回下一个版本号对应的路径。
    例如 base_path = '/maps/g60pro'：
      已有 g60pro_v1.pgm -> 返回 '/maps/g60pro_v2'
      已有 g60pro_v1.pgm, g60pro_v3.pgm -> 返回 '/maps/g60pro_v4'
      没有找到 -> 返回 '/maps/g60pro_v1'
    """
    directory = os.path.dirname(base_path)
    basename = os.path.basename(base_path)
    pattern = re.compile(rf'^{re.escape(basename)}_v(\d+)\.(pgm|yaml|pbstream)$')

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

        # OccupancyGrid 值域：-1=unknown, 0=free, 1~99=中间概率, 100=occupied
        # 阈值与 YAML 保持一致：free_thresh=0.196(≈20), occupied_thresh=0.65(65)
        # 必须一步布尔索引，img[mask1][mask2]=x 是无效的（boolean indexing 返回副本）
        img = np.full((h, w), 205, dtype=np.uint8)
        img[(data != -1) & (data <= 19)] = 254   # free → white
        img[(data != -1) & (data >= 65)] = 0     # occupied → black
        img = np.flipud(img)                     # 图像 y 轴朝上

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

        # 保存 PBSTREAM（Cartographer 纯定位格式）
        pbstream_path = self.save_path + '.pbstream'
        self.save_pbstream(pbstream_path)

        self.get_logger().info(f'地图保存完成: {self.save_path}')

    def save_pbstream(self, pbstream_path):
        """调用 Cartographer /write_state 服务保存 .pbstream"""
        client = self.create_client(WriteState, '/write_state')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/write_state 服务不可用，跳过 pbstream 保存')
            return

        req = WriteState.Request()
        req.filename = pbstream_path
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done() and future.result() is not None:
            self.get_logger().info(f'PBSTREAM 已保存: {pbstream_path}')
        else:
            self.get_logger().warn(f'PBSTREAM 保存失败')


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
