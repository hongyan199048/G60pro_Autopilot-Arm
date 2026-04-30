#!/usr/bin/env python3
"""
检查右前单线激光雷达（LakiBeam1S）的点云角度和距离信息
话题: /lidar/single_1/scan
传感器: 192.168.2.151 (右前)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import math

class SingleLidar1Checker(Node):
    def __init__(self):
        super().__init__('single_lidar_1_checker')

        # 配置 QoS：BEST_EFFORT 以匹配传感器发布者
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar/single_1/scan',
            self.scan_callback,
            qos_profile)
        self.count = 0
        self.get_logger().info('等待右前单线激光雷达数据 (/lidar/single_1/scan)...')

    def scan_callback(self, msg):
        self.count += 1
        if self.count > 1:
            return

        total_points = len(msg.ranges)
        valid_points = []
        inf_points = 0
        zero_points = 0

        # 统计并收集有效点
        for i, r in enumerate(msg.ranges):
            if math.isinf(r):
                inf_points += 1
            elif r == 0.0:
                zero_points += 1
            elif r > msg.range_min and r < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                valid_points.append((i, angle, r))

        print(f"\n{'='*60}")
        print(f"{'右前单线激光雷达 (LakiBeam1S) 数据分析':^60}")
        print(f"{'='*60}")
        print(f"话题: /lidar/single_1/scan")
        print(f"传感器 IP: 192.168.2.151")
        print(f"Frame ID: {msg.header.frame_id}")
        print(f"时间戳: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        print(f"\n{'基本信息':-^60}")
        print(f"总点数: {total_points}")
        print(f"有效距离点数: {len(valid_points)} ({len(valid_points)/total_points*100:.1f}%)")
        print(f"无穷大 (inf) 点数: {inf_points} ({inf_points/total_points*100:.1f}%)")
        print(f"零值点数: {zero_points} ({zero_points/total_points*100:.1f}%)")

        print(f"\n{'测量范围':-^60}")
        print(f"距离范围: {msg.range_min:.2f} - {msg.range_max:.2f} 米")
        print(f"角度范围: {math.degrees(msg.angle_min):.1f}° - {math.degrees(msg.angle_max):.1f}°")
        print(f"角度增量: {math.degrees(msg.angle_increment):.3f}°")
        print(f"扫描时间: {msg.scan_time:.4f} 秒")
        print(f"扫描频率: {1.0/msg.scan_time:.1f} Hz" if msg.scan_time > 0 else "扫描频率: N/A")

        if valid_points:
            distances = [r for _, _, r in valid_points]
            print(f"\n{'有效距离统计':-^60}")
            print(f"最小距离: {min(distances):.3f} 米")
            print(f"最大距离: {max(distances):.3f} 米")
            print(f"平均距离: {sum(distances)/len(distances):.3f} 米")

            print(f"\n{'前 20 个有效点 (索引, 角度, 距离)':-^60}")
            print(f"{'索引':<8} {'角度(°)':<12} {'距离(m)':<12}")
            print(f"{'-'*60}")
            for idx, angle, dist in valid_points[:20]:
                print(f"{idx:<8} {math.degrees(angle):>10.2f}° {dist:>10.3f}m")

            if len(valid_points) > 20:
                print(f"\n{'最后 10 个有效点':-^60}")
                print(f"{'索引':<8} {'角度(°)':<12} {'距离(m)':<12}")
                print(f"{'-'*60}")
                for idx, angle, dist in valid_points[-10:]:
                    print(f"{idx:<8} {math.degrees(angle):>10.2f}° {dist:>10.3f}m")

            # 按距离分段统计
            print(f"\n{'距离分段统计':-^60}")
            ranges_bins = [
                (0.0, 0.5, "0.0-0.5m (近场)"),
                (0.5, 1.0, "0.5-1.0m"),
                (1.0, 2.0, "1.0-2.0m"),
                (2.0, 5.0, "2.0-5.0m"),
                (5.0, 10.0, "5.0-10.0m (远场)")
            ]
            for min_r, max_r, label in ranges_bins:
                count = sum(1 for _, _, r in valid_points if min_r <= r < max_r)
                if count > 0:
                    print(f"{label:<20}: {count:>4} 点 ({count/len(valid_points)*100:>5.1f}%)")
        else:
            print(f"\n{'⚠️  警告':-^60}")
            print("没有检测到有效的距离数据！")
            print("可能原因:")
            print("  1. 雷达未启动或未连接")
            print("  2. 网络配置错误 (检查 192.168.2.151)")
            print("  3. 雷达视野内无障碍物")

        print(f"{'='*60}\n")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = SingleLidar1Checker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()