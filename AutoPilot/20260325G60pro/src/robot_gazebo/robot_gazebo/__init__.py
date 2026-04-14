#!/usr/bin/env python3
"""
G60Pro Gazebo 仿真节点
提供仿真模式的 CAN 通信和底盘控制
"""

import rclpy
from rclpy.node import Node


class GazeboNode(Node):
    """Gazebo 仿真节点"""

    def __init__(self):
        super().__init__('robot_gazebo')
        self.get_logger().info('Gazebo 仿真节点已启动')

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()