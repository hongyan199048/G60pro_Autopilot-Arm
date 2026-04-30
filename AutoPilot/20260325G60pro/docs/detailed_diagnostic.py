#!/usr/bin/env python3
"""
详细诊断单线雷达在 costmap 中的处理情况
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
import time

class DetailedDiagnostic(Node):
    def __init__(self):
        super().__init__('detailed_diagnostic')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅单线雷达
        qos_sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan1_sub = self.create_subscription(
            LaserScan, '/lidar/single_1/scan', self.scan1_callback, qos_sensor)

        # 订阅 costmap
        qos_costmap = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, qos_costmap)

        self.scan_count = 0
        self.costmap_count = 0
        self.last_scan_time = None
        self.last_costmap_time = None

        self.get_logger().info("=== 开始详细诊断 ===")

    def scan1_callback(self, msg):
        self.scan_count += 1
        self.last_scan_time = time.time()

        if self.scan_count <= 3:
            valid = sum(1 for r in msg.ranges if 0.1 < r < 50.0)
            self.get_logger().info(
                f"\n[单线雷达1 #{self.scan_count}]"
                f"\n  frame_id: {msg.header.frame_id}"
                f"\n  timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
                f"\n  valid_points: {valid}/{len(msg.ranges)}"
            )

            # 检查 TF
            try:
                trans = self.tf_buffer.lookup_transform(
                    'base_footprint',
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                self.get_logger().info(
                    f"  TF (latest): OK - x={trans.transform.translation.x:.3f}, "
                    f"y={trans.transform.translation.y:.3f}, z={trans.transform.translation.z:.3f}")
            except Exception as e:
                self.get_logger().error(f"  TF (latest): FAILED - {str(e)[:100]}")

    def costmap_callback(self, msg):
        self.costmap_count += 1
        self.last_costmap_time = time.time()

        if self.costmap_count <= 5:
            occupied = sum(1 for c in msg.data if c > 50)
            inflated = sum(1 for c in msg.data if 1 <= c <= 50)

            self.get_logger().info(
                f"\n[Costmap #{self.costmap_count}]"
                f"\n  timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
                f"\n  occupied: {occupied}, inflated: {inflated}"
                f"\n  time_since_last_scan: {time.time() - self.last_scan_time if self.last_scan_time else 'N/A':.3f}s"
            )

        if self.costmap_count >= 5:
            self.get_logger().info(
                f"\n=== 诊断完成 ==="
                f"\n  单线雷达帧数: {self.scan_count}"
                f"\n  Costmap 更新次数: {self.costmap_count}"
            )
            raise SystemExit

def main():
    rclpy.init()
    node = DetailedDiagnostic()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()