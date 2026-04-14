#!/usr/bin/env python3
"""
G60Pro CAN 通信节点
负责与 RDM 控制器通信，通过 CAN1 发送指令，CAN4 接收电机状态
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from robot_msgs.msg import MotorCmd, MotorState, CanFrame

import os
import can
import struct
import time


class CanNode(Node):
    """CAN 通信节点"""

    def __init__(self):
        super().__init__('robot_can_node')

        # 声明参数
        self.declare_parameter('can1_channel', 'can0')
        self.declare_parameter('can4_channel', 'can1')
        self.declare_parameter('can1_baudrate', 500000)
        self.declare_parameter('can4_baudrate', 1000000)
        self.declare_parameter('use_sim', False)

        self.can1_channel = self.get_parameter('can1_channel').value
        self.can4_channel = self.get_parameter('can4_channel').value
        self.use_sim = self.get_parameter('use_sim').value

        # CAN 总线
        self.bus_can1 = None
        self.bus_can4 = None

        # 初始化 CAN 总线
        self._init_can_buses()

        # 订阅 motor_cmd
        self.motor_cmd_sub = self.create_subscription(
            MotorCmd,
            'motor_cmd',
            self._motor_cmd_callback,
            QoSProfile(depth=10)
        )

        # 发布 motor_state
        self.motor_state_pub = self.create_publisher(
            MotorState,
            'motor_state',
            QoSProfile(depth=10)
        )

        # 发布 CAN 帧（用于调试）
        self.can_frame_pub = self.create_publisher(
            CanFrame,
            'can_frames',
            QoSProfile(depth=100)
        )

        # 定时器 - 定期检查 CAN 接收
        self.timer = self.create_timer(0.01, self._can_receive_callback)

        self.get_logger().info('CAN 节点已启动')

    def _init_can_buses(self):
        """初始化 CAN 总线"""
        if self.use_sim:
            self.get_logger().info('模拟模式启用，跳过 CAN 初始化')
            return

        try:
            # CAN1 - 连接 RDM 控制器
            self.bus_can1 = can.interface.Bus(
                channel=self.can1_channel,
                bustype='socketcan'
            )
            self.get_logger().info(f'CAN1 已连接: {self.can1_channel}')
        except Exception as e:
            self.get_logger().error(f'CAN1 连接失败: {e}')
            self.bus_can1 = None

        try:
            # CAN4 - 连接电机驱动器
            self.bus_can4 = can.interface.Bus(
                channel=self.can4_channel,
                bustype='socketcan'
            )
            self.get_logger().info(f'CAN4 已连接: {self.can4_channel}')
        except Exception as e:
            self.get_logger().error(f'CAN4 连接失败: {e}')
            self.bus_can4 = None

    def _motor_cmd_callback(self, msg: MotorCmd):
        """处理电机指令，发送到 CAN1"""
        # 4轮8驱：4个转向角度 + 4个驱动速度
        steer_angles = msg.steer_angle  # 4个转向角度
        drive_velocities = msg.drive_velocity  # 4个驱动速度

        # 构建 CAN 帧并发送到 CAN1
        # 注意：具体的 DBC 消息格式需要根据实际协议实现
        self._send_can1_cmd(steer_angles, drive_velocities)

    def _send_can1_cmd(self, steer_angles, drive_velocities):
        """发送指令到 CAN1 (RDM 控制器)"""
        if self.bus_can1 is None:
            return

        try:
            # 打包数据：根据 DBC 协议格式
            # 示例：0x101 - 转向角度 + 驱动速度命令
            data = struct.pack('<8f',
                              steer_angles[0], steer_angles[1], steer_angles[2], steer_angles[3],
                              drive_velocities[0], drive_velocities[1], drive_velocities[2], drive_velocities[3])

            msg = can.Message(
                arbitration_id=0x101,
                data=data,
                is_extended_id=False
            )
            self.bus_can1.send(msg)

        except Exception as e:
            self.get_logger().error(f'CAN1 发送失败: {e}')

    def _can_receive_callback(self):
        """定期接收 CAN4 数据"""
        if self.bus_can4 is None:
            return

        try:
            # 接收 CAN4 消息（电机状态反馈）
            while True:
                msg = self.bus_can4.recv(timeout=0.001)
                if msg is None:
                    break

                # 解析电机状态
                self._parse_motor_state(msg)

                # 发布 CAN 帧用于调试
                self._publish_can_frame(msg)

        except Exception as e:
            self.get_logger().error(f'CAN4 接收错误: {e}')

    def _parse_motor_state(self, msg: can.Message):
        """解析电机状态消息"""
        # 根据 DBC 协议解析
        # 示例：0x201 - 电机状态反馈
        if msg.arbitration_id == 0x201:
            try:
                state_msg = MotorState()
                state_msg.header.stamp = self.get_clock().now().to_msg()
                state_msg.header.frame_id = 'base_link'

                # 解析数据（示例：根据实际协议调整）
                if len(msg.data) >= 16:
                    steer_feedback = struct.unpack('<4f', msg.data[0:16])
                    state_msg.steer_angle_feedback = list(steer_feedback)

                if len(msg.data) >= 32:
                    drive_feedback = struct.unpack('<4f', msg.data[16:32])
                    state_msg.drive_velocity_feedback = list(drive_feedback)

                # 温度、电流等（示例值）
                state_msg.temperature = [0.0] * 4
                state_msg.current = [0.0] * 4
                state_msg.voltage = 24.0
                state_msg.error_code = 0

                self.motor_state_pub.publish(state_msg)

            except Exception as e:
                self.get_logger().error(f'解析电机状态失败: {e}')

    def _publish_can_frame(self, msg: can.Message):
        """发布 CAN 帧用于调试"""
        can_frame = CanFrame()
        can_frame.can_id = msg.arbitration_id
        can_frame.dlc = msg.dlc
        can_frame.data = list(msg.data)
        can_frame.stamp = self.get_clock().now().to_msg()

        self.can_frame_pub.publish(can_frame)

    def destroy_node(self):
        """清理资源"""
        if self.bus_can1:
            self.bus_can1.shutdown()
        if self.bus_can4:
            self.bus_can4.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()