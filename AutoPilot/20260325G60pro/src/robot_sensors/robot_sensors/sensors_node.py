#!/usr/bin/env python3
"""
G60Pro 传感器管理节点
启动和管理所有传感器驱动
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


class SensorsNode(Node):
    """传感器管理节点"""

    def __init__(self):
        super().__init__('robot_sensors_node')

        # 声明参数
        self.declare_parameter('enable_lidar16', True)
        self.declare_parameter('enable_single_lidar', True)
        self.declare_parameter('enable_rgbd', True)
        self.declare_parameter('enable_imu', True)

        self.enable_lidar16 = self.get_parameter('enable_lidar16').value
        self.enable_single_lidar = self.get_parameter('enable_single_lidar').value
        self.enable_rgbd = self.get_parameter('enable_rgbd').value
        self.enable_imu = self.get_parameter('enable_imu').value

        self.get_logger().info('传感器管理节点已启动')

        if self.enable_lidar16:
            self.get_logger().info('启用 16线激光雷达 (Helios16)')

        if self.enable_single_lidar:
            self.get_logger().info('启用 单线激光雷达 (LakiBeam1s x2)')

        if self.enable_rgbd:
            self.get_logger().info('启用 RGBD相机 (DaBai DCW2-DW2 x4)')

        if self.enable_imu:
            self.get_logger().info('启用 IMU')


def main(args=None):
    rclpy.init(args=args)
    node = SensorsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()