#!/usr/bin/env python3
"""
Initial Pose Relay Node — 处理 RViz 2D Pose Estimate

功能：
  1. 订阅 /initialpose（RViz 发布的初始位姿）
  2. 调用 Cartographer 的 /finish_trajectory + /start_trajectory 重置定位

原理：
  Cartographer 内部维护一条轨迹记录历史位姿。定位丢失后，内部状态卡在错误位置，
  scan matching 从错误初始位姿出发，搜索窗口覆盖不到正确位置，就一直偏。
  必须通过 finish_trajectory + start_trajectory 清除错误状态，用指定位姿重新开始。
  不发布 TF，避免与 Cartographer 的 map→base_footprint 冲突导致跳变。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory
import math


class InitialPoseRelay(Node):
    def __init__(self):
        super().__init__('initial_pose_relay')

        # 参数：Cartographer 配置路径
        self.declare_parameter('configuration_directory', '')
        self.declare_parameter('configuration_basename', 'cartographer_real_localization.lua')

        # 订阅 RViz 的 /initialpose
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )

        # Cartographer 服务客户端
        self.finish_client = self.create_client(FinishTrajectory, '/finish_trajectory')
        self.start_client = self.create_client(StartTrajectory, '/start_trajectory')

        self.get_logger().info('Initial Pose Relay 已启动（轨迹重置模式），等待 /initialpose...')

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        """接收 RViz 的初始位姿，重置 Cartographer 轨迹"""
        yaw = self.get_yaw(msg.pose.pose.orientation)
        self.get_logger().info(
            f'收到初始位姿: x={msg.pose.pose.position.x:.2f}, '
            f'y={msg.pose.pose.position.y:.2f}, '
            f'yaw={yaw:.2f} rad ({math.degrees(yaw):.1f}°)'
        )

        self.reset_cartographer(msg.pose.pose)

    def reset_cartographer(self, initial_pose: Pose):
        """finish_trajectory → start_trajectory，用指定位姿重新定位"""
        config_dir = self.get_parameter('configuration_directory').get_parameter_value().string_value
        config_basename = self.get_parameter('configuration_basename').get_parameter_value().string_value

        if not config_dir:
            self.get_logger().error('configuration_directory 参数未设置，无法重置 Cartographer')
            return

        # 等待服务可用
        if not self.finish_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/finish_trajectory 服务不可用')
            return
        if not self.start_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/start_trajectory 服务不可用')
            return

        # Step 1: 结束当前轨迹（trajectory_id=0）
        self.get_logger().info('重置 Cartographer: 结束当前轨迹...')
        finish_req = FinishTrajectory.Request()
        finish_req.trajectory_id = 0
        finish_future = self.finish_client.call_async(finish_req)
        finish_future.add_done_callback(
            lambda f: self._on_finish_done(f, config_dir, config_basename, initial_pose)
        )

    def _on_finish_done(self, future, config_dir, config_basename, initial_pose):
        """finish_trajectory 回调：启动新轨迹"""
        try:
            result = future.result()
            self.get_logger().info(f'轨迹已结束: {result.status.message}')
        except Exception as e:
            self.get_logger().error(f'finish_trajectory 失败: {e}')
            return

        # Step 2: 用指定初始位姿启动新轨迹
        self.get_logger().info('重置 Cartographer: 用新位姿启动轨迹...')
        start_req = StartTrajectory.Request()
        start_req.configuration_directory = config_dir
        start_req.configuration_basename = config_basename
        start_req.use_initial_pose = True
        start_req.initial_pose = initial_pose
        start_req.relative_to_trajectory_id = 0

        start_future = self.start_client.call_async(start_req)
        start_future.add_done_callback(self._on_start_done)

    def _on_start_done(self, future):
        """start_trajectory 回调"""
        try:
            result = future.result()
            self.get_logger().info(f'新轨迹已启动 (id={result.trajectory_id}): {result.status.message}')
        except Exception as e:
            self.get_logger().error(f'start_trajectory 失败: {e}')

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
