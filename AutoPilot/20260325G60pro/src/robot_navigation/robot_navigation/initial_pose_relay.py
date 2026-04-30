#!/usr/bin/env python3
"""
Initial Pose Relay Node — 处理 RViz 2D Pose Estimate

功能：
  1. 订阅 /initialpose（RViz 发布的初始位姿）
  2. 持续发布 TF：map → base_footprint（覆盖 Cartographer 的定位结果）
  3. 发布 10 秒，给 Cartographer 足够时间从新位置重新收敛

原理：
  Cartographer 纯定位模式下，内部轨迹状态可能卡在错误位置。
  通过发布 TF 覆盖，强制机器人在 RViz 中显示在正确位置，
  Cartographer 的 scan matching 会逐渐从正确位置附近重新匹配。
  不调用 finish/start_trajectory，避免破坏 pbstream 冻结子图的关联。
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
        self.max_publish_count = 500  # 10 秒 @ 50Hz

        self.get_logger().info('Initial Pose Relay 已启动（TF 覆盖模式，10秒），等待 /initialpose...')

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        """接收 RViz 的初始位姿"""
        yaw = self.get_yaw(msg.pose.pose.orientation)
        self.get_logger().info(
            f'收到初始位姿: x={msg.pose.pose.position.x:.2f}, '
            f'y={msg.pose.pose.position.y:.2f}, '
            f'yaw={yaw:.2f} rad ({math.degrees(yaw):.1f}°)'
        )

        # 保存位姿
        self.current_pose = msg
        self.publish_count = 0

        # 启动 TF 发布定时器（50Hz，发布 10 秒）
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(0.02, self.publish_tf)

    def publish_tf(self):
        """持续发布 TF：map → base_footprint"""
        if self.current_pose is None:
            return

        if self.publish_count >= self.max_publish_count:
            self.get_logger().info('TF 发布完成（10秒），Cartographer 应已接管定位')
            self.timer.cancel()
            self.timer = None
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.current_pose.pose.pose.position.x
        t.transform.translation.y = self.current_pose.pose.pose.position.y
        t.transform.translation.z = self.current_pose.pose.pose.position.z
        t.transform.rotation = self.current_pose.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)
        self.publish_count += 1

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
