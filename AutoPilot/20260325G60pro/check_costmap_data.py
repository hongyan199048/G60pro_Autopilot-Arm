#!/usr/bin/env python3
"""
检查代价地图原始数据，判断残留障碍物是真实存在还是显示问题
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np


class CostmapDataChecker(Node):
    def __init__(self):
        super().__init__('costmap_data_checker')

        # 代价地图 QoS
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 订阅局部代价地图
        self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            costmap_qos
        )

        self.get_logger().info('代价地图数据检查器启动')
        self.get_logger().info('订阅话题: /local_costmap/costmap')
        self.get_logger().info('等待代价地图数据...')

    def costmap_callback(self, msg):
        """分析代价地图数据"""
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # 转换为 numpy 数组
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # 统计各代价值的栅格数量
        free_cells = np.sum(data == 0)           # 自由空间
        unknown_cells = np.sum(data == -1)       # 未知区域
        lethal_cells = np.sum(data == 254)       # 致命障碍
        inscribed_cells = np.sum(data == 253)    # 内切障碍
        inflation_cells = np.sum((data > 0) & (data < 253))  # 膨胀区

        total_cells = width * height

        self.get_logger().info('\n' + '='*80)
        self.get_logger().info('代价地图数据分析')
        self.get_logger().info('='*80)
        self.get_logger().info(f'地图尺寸: {width}×{height} = {total_cells} 栅格')
        self.get_logger().info(f'分辨率: {resolution:.3f} m/栅格')
        self.get_logger().info(f'原点: ({origin_x:.2f}, {origin_y:.2f})')
        self.get_logger().info('')
        self.get_logger().info('代价值分布:')
        self.get_logger().info(f'  自由空间 (cost=0):     {free_cells:6d} ({free_cells/total_cells*100:5.1f}%)')
        self.get_logger().info(f'  未知区域 (cost=-1):    {unknown_cells:6d} ({unknown_cells/total_cells*100:5.1f}%)')
        self.get_logger().info(f'  膨胀区 (cost=1-252):   {inflation_cells:6d} ({inflation_cells/total_cells*100:5.1f}%)')
        self.get_logger().info(f'  内切障碍 (cost=253):   {inscribed_cells:6d} ({inscribed_cells/total_cells*100:5.1f}%)')
        self.get_logger().info(f'  致命障碍 (cost=254):   {lethal_cells:6d} ({lethal_cells/total_cells*100:5.1f}%)')
        self.get_logger().info('')

        # 找出所有致命障碍的位置
        lethal_positions = np.argwhere(data == 254)
        if len(lethal_positions) > 0:
            self.get_logger().info(f'致命障碍物位置（前 10 个）:')
            for i, (row, col) in enumerate(lethal_positions[:10]):
                # 转换为世界坐标
                world_x = origin_x + col * resolution
                world_y = origin_y + row * resolution
                self.get_logger().info(f'  #{i+1}: 栅格({row}, {col}) → 世界坐标({world_x:.2f}, {world_y:.2f})')

            if len(lethal_positions) > 10:
                self.get_logger().info(f'  ... 还有 {len(lethal_positions)-10} 个障碍物')
        else:
            self.get_logger().info('✓ 无致命障碍物')

        # 检查是否有异常的障碍物分布
        if lethal_cells > 0:
            # 计算障碍物的连通区域
            from scipy import ndimage
            labeled_array, num_features = ndimage.label(data == 254)

            self.get_logger().info('')
            self.get_logger().info(f'障碍物连通区域数量: {num_features}')

            # 统计每个连通区域的大小
            region_sizes = []
            for i in range(1, num_features + 1):
                size = np.sum(labeled_array == i)
                region_sizes.append(size)

            region_sizes.sort(reverse=True)
            self.get_logger().info(f'最大连通区域: {region_sizes[0]} 栅格')
            self.get_logger().info(f'前 5 大连通区域: {region_sizes[:5]}')

            # 检查是否有很多小的孤立障碍物（可能是残留）
            small_regions = [s for s in region_sizes if s < 5]
            if len(small_regions) > 10:
                self.get_logger().warn(f'⚠ 检测到 {len(small_regions)} 个小孤立障碍物（<5栅格），可能是残留')

        self.get_logger().info('='*80 + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = CostmapDataChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
