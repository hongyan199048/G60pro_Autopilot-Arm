#!/usr/bin/env python3
"""
检查单线雷达数据是否被 costmap 正确处理
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import sys

class CostmapChecker(Node):
    def __init__(self):
        super().__init__('costmap_checker')

        # 订阅单线雷达数据
        qos_sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan1_sub = self.create_subscription(
            LaserScan, '/lidar/single_1/scan', self.scan1_callback, qos_sensor)
        self.scan2_sub = self.create_subscription(
            LaserScan, '/lidar/single_2/scan', self.scan2_callback, qos_sensor)

        # 订阅 costmap
        qos_costmap = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.local_costmap_callback, qos_costmap)
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.global_costmap_callback, qos_costmap)

        self.scan1_count = 0
        self.scan2_count = 0
        self.local_costmap_count = 0
        self.global_costmap_count = 0

        self.get_logger().info("开始监控单线雷达和 costmap...")

    def scan1_callback(self, msg):
        valid = sum(1 for r in msg.ranges if 0.1 < r < 50.0)
        self.scan1_count += 1
        if self.scan1_count <= 3:
            self.get_logger().info(f"[单线雷达1] Frame {self.scan1_count}: "
                                   f"total={len(msg.ranges)}, valid={valid}, "
                                   f"frame_id={msg.header.frame_id}")

    def scan2_callback(self, msg):
        valid = sum(1 for r in msg.ranges if 0.1 < r < 50.0)
        self.scan2_count += 1
        if self.scan2_count <= 3:
            self.get_logger().info(f"[单线雷达2] Frame {self.scan2_count}: "
                                   f"total={len(msg.ranges)}, valid={valid}, "
                                   f"frame_id={msg.header.frame_id}")

    def local_costmap_callback(self, msg):
        self.local_costmap_count += 1
        if self.local_costmap_count <= 3:
            occupied = sum(1 for cell in msg.data if cell > 50)
            inflated = sum(1 for cell in msg.data if 1 <= cell <= 50)
            free = sum(1 for cell in msg.data if cell == 0)
            unknown = sum(1 for cell in msg.data if cell < 0)

            self.get_logger().info(f"[Local Costmap] Update {self.local_costmap_count}: "
                                   f"size={msg.info.width}x{msg.info.height}, "
                                   f"resolution={msg.info.resolution:.3f}m, "
                                   f"occupied={occupied}, inflated={inflated}, "
                                   f"free={free}, unknown={unknown}")

    def global_costmap_callback(self, msg):
        self.global_costmap_count += 1
        if self.global_costmap_count <= 3:
            occupied = sum(1 for cell in msg.data if cell > 50)
            inflated = sum(1 for cell in msg.data if 1 <= cell <= 50)

            self.get_logger().info(f"[Global Costmap] Update {self.global_costmap_count}: "
                                   f"size={msg.info.width}x{msg.info.height}, "
                                   f"occupied={occupied}, inflated={inflated}")

        # 收集足够数据后退出
        if (self.scan1_count >= 3 and self.scan2_count >= 3 and
            self.local_costmap_count >= 3 and self.global_costmap_count >= 3):
            self.get_logger().info("\n=== 总结 ===")
            self.get_logger().info(f"单线雷达1: {self.scan1_count} 帧")
            self.get_logger().info(f"单线雷达2: {self.scan2_count} 帧")
            self.get_logger().info(f"Local Costmap: {self.local_costmap_count} 次更新")
            self.get_logger().info(f"Global Costmap: {self.global_costmap_count} 次更新")
            sys.exit(0)

def main():
    rclpy.init()
    node = CostmapChecker()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()