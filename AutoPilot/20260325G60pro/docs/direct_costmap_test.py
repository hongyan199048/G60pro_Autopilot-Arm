#!/usr/bin/env python3
"""
绕过所有配置检查，直接模拟 ObstacleLayer 处理单线雷达数据
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import math

class DirectCostmapTest(Node):
    def __init__(self):
        super().__init__('direct_costmap_test')

        # 订阅单线雷达
        qos_sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(
            LaserScan, '/lidar/single_1/scan', self.scan_callback, qos_sensor)

        # 发布测试 costmap
        qos_costmap = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/test_costmap', qos_costmap)

        self.get_logger().info("=== 直接 Costmap 测试 ===")
        self.get_logger().info("订阅: /lidar/single_1/scan")
        self.get_logger().info("发布: /test_costmap")

    def scan_callback(self, scan_msg):
        """直接将 LaserScan 转换成 OccupancyGrid"""
        # 创建一个 10m x 10m 的 costmap（分辨率 0.05m = 200x200 cells）
        resolution = 0.05
        width = 200
        height = 200

        costmap = OccupancyGrid()
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.header.frame_id = 'base_footprint'
        costmap.info.resolution = resolution
        costmap.info.width = width
        costmap.info.height = height
        costmap.info.origin.position.x = -5.0  # 中心在 (0, 0)
        costmap.info.origin.position.y = -5.0
        costmap.info.origin.position.z = 0.0
        costmap.info.origin.orientation.w = 1.0

        # 初始化为 free (0)
        data = np.zeros(width * height, dtype=np.int8)

        # 将 LaserScan 点标记为 occupied (100)
        angle = scan_msg.angle_min
        valid_count = 0
        marked_count = 0

        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max and not math.isinf(r):
                valid_count += 1

                # 转换到 base_footprint 坐标系（假设雷达在车前方）
                x = r * math.cos(angle) + 0.738  # 雷达 X 偏移
                y = r * math.sin(angle) - 0.340  # 雷达 Y 偏移

                # 转换到 costmap 坐标
                mx = int((x - costmap.info.origin.position.x) / resolution)
                my = int((y - costmap.info.origin.position.y) / resolution)

                # 标记为 occupied
                if 0 <= mx < width and 0 <= my < height:
                    idx = my * width + mx
                    data[idx] = 100
                    marked_count += 1

            angle += scan_msg.angle_increment

        costmap.data = data.tolist()
        self.costmap_pub.publish(costmap)

        self.get_logger().info(
            f"处理完成: valid_points={valid_count}, marked_cells={marked_count}")

def main():
    rclpy.init()
    node = DirectCostmapTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()