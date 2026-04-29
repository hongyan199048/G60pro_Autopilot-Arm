#!/usr/bin/env python3
"""
避障功能诊断脚本
检查 Nav2 代价地图、障碍物层、传感器数据是否正常
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import time
from collections import defaultdict


class ObstacleAvoidanceChecker(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_checker')

        self.data = defaultdict(lambda: {'count': 0, 'last_time': None, 'last_data': None})

        # QoS 配置：传感器数据使用 BEST_EFFORT（匹配雷达驱动）
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS 配置：代价地图使用 TRANSIENT_LOCAL + RELIABLE（匹配 Nav2）
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 订阅代价地图
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.local_costmap_callback, costmap_qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.global_costmap_callback, costmap_qos)

        # 订阅传感器数据（使用 BEST_EFFORT QoS）
        self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.create_subscription(LaserScan, '/lidar/single_1/scan', self.single1_callback, sensor_qos)
        self.create_subscription(LaserScan, '/lidar/single_2/scan', self.single2_callback, sensor_qos)

        # 定时器：每 2 秒打印统计
        self.create_timer(2.0, self.print_status)

        self.get_logger().info('避障诊断启动，监控话题：')
        self.get_logger().info('  - /local_costmap/costmap')
        self.get_logger().info('  - /global_costmap/costmap')
        self.get_logger().info('  - /scan (Helios16 多线雷达)')
        self.get_logger().info('  - /lidar/single_1/scan (右前单线)')
        self.get_logger().info('  - /lidar/single_2/scan (左后单线)')

    def local_costmap_callback(self, msg):
        self.data['local_costmap']['count'] += 1
        self.data['local_costmap']['last_time'] = time.time()
        # 统计障碍物数量（cost > 0）
        obstacles = sum(1 for c in msg.data if c > 0)
        self.data['local_costmap']['last_data'] = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'obstacles': obstacles,
            'total_cells': len(msg.data)
        }

    def global_costmap_callback(self, msg):
        self.data['global_costmap']['count'] += 1
        self.data['global_costmap']['last_time'] = time.time()
        obstacles = sum(1 for c in msg.data if c > 0)
        self.data['global_costmap']['last_data'] = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'obstacles': obstacles,
            'total_cells': len(msg.data)
        }

    def scan_callback(self, msg):
        self.data['scan']['count'] += 1
        self.data['scan']['last_time'] = time.time()
        # 统计有效点数（非 inf）
        valid_points = sum(1 for r in msg.ranges if r < msg.range_max and r > msg.range_min)
        min_range = min([r for r in msg.ranges if r < msg.range_max and r > msg.range_min], default=0)
        self.data['scan']['last_data'] = {
            'total_points': len(msg.ranges),
            'valid_points': valid_points,
            'min_range': min_range,
            'range_min': msg.range_min,
            'range_max': msg.range_max
        }

    def single1_callback(self, msg):
        self.data['single_1']['count'] += 1
        self.data['single_1']['last_time'] = time.time()
        valid_points = sum(1 for r in msg.ranges if r < msg.range_max and r > msg.range_min)
        min_range = min([r for r in msg.ranges if r < msg.range_max and r > msg.range_min], default=0)
        self.data['single_1']['last_data'] = {
            'total_points': len(msg.ranges),
            'valid_points': valid_points,
            'min_range': min_range,
            'range_min': msg.range_min,
            'range_max': msg.range_max
        }

    def single2_callback(self, msg):
        self.data['single_2']['count'] += 1
        self.data['single_2']['last_time'] = time.time()
        valid_points = sum(1 for r in msg.ranges if r < msg.range_max and r > msg.range_min)
        min_range = min([r for r in msg.ranges if r < msg.range_max and r > msg.range_min], default=0)
        self.data['single_2']['last_data'] = {
            'total_points': len(msg.ranges),
            'valid_points': valid_points,
            'min_range': min_range,
            'range_min': msg.range_min,
            'range_max': msg.range_max
        }

    def print_status(self):
        now = time.time()
        self.get_logger().info('\n' + '='*80)
        self.get_logger().info('避障功能状态检查')
        self.get_logger().info('='*80)

        # 检查传感器数据
        self.get_logger().info('\n【传感器数据】')
        for topic in ['scan', 'single_1', 'single_2']:
            if self.data[topic]['count'] > 0:
                dt = now - self.data[topic]['last_time']
                data = self.data[topic]['last_data']
                status = '✓ 正常' if dt < 1.0 else f'⚠ 延迟 {dt:.1f}s'
                self.get_logger().info(
                    f'  {topic:12s}: {status:15s} | '
                    f'接收 {self.data[topic]["count"]:4d} 帧 | '
                    f'有效点 {data["valid_points"]:4d}/{data["total_points"]:4d} | '
                    f'最近障碍 {data["min_range"]:.2f}m'
                )
            else:
                self.get_logger().warn(f'  {topic:12s}: ✗ 无数据')

        # 检查代价地图
        self.get_logger().info('\n【代价地图】')
        for topic in ['local_costmap', 'global_costmap']:
            if self.data[topic]['count'] > 0:
                dt = now - self.data[topic]['last_time']
                data = self.data[topic]['last_data']
                status = '✓ 正常' if dt < 2.0 else f'⚠ 延迟 {dt:.1f}s'
                obstacle_ratio = data['obstacles'] / data['total_cells'] * 100
                self.get_logger().info(
                    f'  {topic:15s}: {status:15s} | '
                    f'更新 {self.data[topic]["count"]:4d} 次 | '
                    f'尺寸 {data["width"]}x{data["height"]} | '
                    f'分辨率 {data["resolution"]:.2f}m | '
                    f'障碍物 {data["obstacles"]:6d} ({obstacle_ratio:.1f}%)'
                )
            else:
                self.get_logger().warn(f'  {topic:15s}: ✗ 无数据')

        # 诊断建议
        self.get_logger().info('\n【诊断建议】')
        issues = []

        # 检查传感器是否有数据
        for topic in ['scan', 'single_1', 'single_2']:
            if self.data[topic]['count'] == 0:
                issues.append(f'⚠ {topic} 无数据，检查传感器驱动是否启动')
            elif self.data[topic]['last_data']['valid_points'] == 0:
                issues.append(f'⚠ {topic} 无有效点，检查传感器是否被遮挡')

        # 检查代价地图是否更新
        for topic in ['local_costmap', 'global_costmap']:
            if self.data[topic]['count'] == 0:
                issues.append(f'⚠ {topic} 无数据，检查 Nav2 是否启动')
            elif self.data[topic]['last_time'] and (now - self.data[topic]['last_time']) > 5.0:
                issues.append(f'⚠ {topic} 长时间未更新，检查 Nav2 节点状态')

        # 检查代价地图是否有障碍物
        if self.data['local_costmap']['count'] > 0:
            data = self.data['local_costmap']['last_data']
            if data['obstacles'] == 0:
                issues.append('⚠ 局部代价地图无障碍物，可能传感器数据未正确融合')

        if issues:
            for issue in issues:
                self.get_logger().warn(f'  {issue}')
        else:
            self.get_logger().info('  ✓ 避障功能正常')

        self.get_logger().info('='*80 + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
