#!/usr/bin/env python3
"""
分析 Nav2 全局路径的转弯半径
订阅 /plan 话题，计算路径中每个点的曲率半径，输出最小转弯半径
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import math
import numpy as np


class PathAnalyzer(Node):
    def __init__(self):
        super().__init__('path_analyzer')
        self.sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        self.get_logger().info('路径分析器已启动，等待 /plan 话题...')
        self.get_logger().info('提示：在 RViz 中发送导航目标后，会自动分析路径')

    def path_callback(self, msg: Path):
        """分析路径"""
        if len(msg.poses) < 3:
            self.get_logger().warn(f'路径点太少（{len(msg.poses)}），无法分析曲率')
            return

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'收到新路径，共 {len(msg.poses)} 个点')

        # 提取路径点坐标
        points = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            points.append((x, y))

        # 计算每个点的曲率半径（使用三点法）
        radii = []
        for i in range(1, len(points) - 1):
            p1 = np.array(points[i - 1])
            p2 = np.array(points[i])
            p3 = np.array(points[i + 1])

            # 过滤距离太近的点（避免计算误差）
            dist_12 = np.linalg.norm(p2 - p1)
            dist_23 = np.linalg.norm(p3 - p2)
            if dist_12 < 0.1 or dist_23 < 0.1:  # 相邻点距离 < 10cm，跳过
                continue

            # 三点确定圆，计算曲率半径
            radius = self.calculate_radius(p1, p2, p3)
            if radius is not None and 0.1 < radius < 1000:  # 过滤极小和极大值
                radii.append(radius)

        if not radii:
            self.get_logger().info('路径为直线，无转弯')
            return

        # 统计
        min_radius = min(radii)
        max_radius = max(radii)
        avg_radius = np.mean(radii)
        median_radius = np.median(radii)

        # 统计倒车段（改进判断逻辑）
        reverse_count = 0
        forward_count = 0
        for i in range(1, len(msg.poses)):
            # 通过路径方向判断是否倒车
            dx = msg.poses[i].pose.position.x - msg.poses[i-1].pose.position.x
            dy = msg.poses[i].pose.position.y - msg.poses[i-1].pose.position.y

            # 过滤距离太近的点
            dist = math.sqrt(dx**2 + dy**2)
            if dist < 0.05:  # 相邻点距离 < 5cm，跳过
                continue

            # 获取前一个点的朝向（车头方向）
            q = msg.poses[i-1].pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

            # 路径方向
            path_yaw = math.atan2(dy, dx)

            # 计算车头方向与路径方向的夹角
            angle_diff = abs(self.normalize_angle(path_yaw - yaw))

            # 夹角 > 90° 为倒车，< 90° 为前进
            if angle_diff > math.pi / 2:
                reverse_count += 1
            else:
                forward_count += 1

        total_valid = forward_count + reverse_count
        if total_valid > 0:
            reverse_ratio = reverse_count / total_valid * 100
            forward_ratio = forward_count / total_valid * 100
        else:
            reverse_ratio = 0
            forward_ratio = 0

        self.get_logger().info('-' * 60)
        self.get_logger().info('路径统计：')
        self.get_logger().info(f'  总点数: {len(msg.poses)}')
        self.get_logger().info(f'  有效转弯段: {len(radii)} 段（已过滤距离 < 10cm 的点）')
        self.get_logger().info(f'  前进段: {forward_count} ({forward_ratio:.1f}%)')
        self.get_logger().info(f'  倒车段: {reverse_count} ({reverse_ratio:.1f}%)')
        self.get_logger().info('')
        self.get_logger().info('转弯半径统计：')
        self.get_logger().info(f'  最小转弯半径: {min_radius:.3f} m  ← 关键指标')
        self.get_logger().info(f'  最大转弯半径: {max_radius:.3f} m')
        self.get_logger().info(f'  平均转弯半径: {avg_radius:.3f} m')
        self.get_logger().info(f'  中位转弯半径: {median_radius:.3f} m')
        self.get_logger().info('')

        # 与配置对比
        config_min_radius = 2.0  # 从配置文件读取
        if min_radius < config_min_radius:
            self.get_logger().warn(
                f'⚠️  实际最小转弯半径 {min_radius:.3f}m < 配置值 {config_min_radius}m'
            )
        else:
            self.get_logger().info(
                f'✓  实际最小转弯半径 {min_radius:.3f}m >= 配置值 {config_min_radius}m'
            )

        self.get_logger().info('=' * 60)

    @staticmethod
    def calculate_radius(p1, p2, p3):
        """
        三点法计算曲率半径
        返回 None 表示三点共线（直线段）
        """
        # 计算三角形三边长
        a = np.linalg.norm(p2 - p1)
        b = np.linalg.norm(p3 - p2)
        c = np.linalg.norm(p3 - p1)

        # 半周长
        s = (a + b + c) / 2

        # 面积（海伦公式）
        area_sq = s * (s - a) * (s - b) * (s - c)
        if area_sq <= 0:
            return None  # 三点共线

        area = math.sqrt(area_sq)

        # 外接圆半径 R = abc / (4 * Area)
        if area < 1e-6:
            return None
        radius = (a * b * c) / (4 * area)

        return radius

    @staticmethod
    def normalize_angle(angle):
        """归一化角度到 [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = PathAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
