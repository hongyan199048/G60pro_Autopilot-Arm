#!/usr/bin/env python3
"""
Initial Pose Relay Node — 处理 RViz 2D Pose Estimate

功能：
  1. 订阅 /initialpose（RViz 发布的初始位姿）
  2. 发布 TF：map → base_footprint（覆盖 Cartographer 的定位结果）
  3. 持续发布 3 秒，让 Cartographer 有时间重新对齐

使用场景：
  - Cartographer 纯定位模式下，初始位置偏差大
  - 用户在 RViz 中点击 "2D Pose Estimate" 手动校正位置
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import math


class InitialPoseRelay(Node):
    def __init__(self):
        super().__init__('initial_pose_relay')

        # 订阅 RViz 的 /initialpose
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )

        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 定时器（用于持续发布 TF）
        self.timer = None
        self.current_pose = None
        self.publish_count = 0
        self.max_publish_count = 150  # 3秒 @ 50Hz

        self.get_logger().info('Initial Pose Relay 已启动，等待 /initialpose...')

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        """接收 RViz 的初始位姿"""
        self.get_logger().info(
            f'收到初始位姿: x={msg.pose.pose.position.x:.2f}, '
            f'y={msg.pose.pose.position.y:.2f}, '
            f'yaw={self.get_yaw(msg.pose.pose.orientation):.2f} rad'
        )

        # 保存位姿
        self.current_pose = msg
        self.publish_count = 0

        # 启动定时器（50Hz 发布 TF）
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(0.02, self.publish_tf)

    def publish_tf(self):
        """持续发布 TF：map → base_footprint"""
        if self.current_pose is None:
            return

        if self.publish_count >= self.max_publish_count:
            self.get_logger().info('TF 发布完成（3秒），Cartographer 应已重新对齐')
            self.timer.cancel()
            self.timer = None
            return

        # 构造 TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.current_pose.pose.pose.position.x
        t.transform.translation.y = self.current_pose.pose.pose.position.y
        t.transform.translation.z = self.current_pose.pose.pose.position.z

        t.transform.rotation = self.current_pose.pose.pose.orientation

        # 发布 TF
        self.tf_broadcaster.sendTransform(t)

        self.publish_count += 1

        # 每 50 次（1秒）打印一次进度
        if self.publish_count % 50 == 0:
            self.get_logger().info(f'TF 发布中... ({self.publish_count}/{self.max_publish_count})')

    @staticmethod
    def get_yaw(q):
        """从四元数提取 yaw 角"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
