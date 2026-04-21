#!/usr/bin/env python3
"""
G60Pro CAN 通信节点
基于 DBC (G60_CAN1_RDM_V1.0.dbc) 精确编解码

工控机 → RDM (Tx):
  0x210 LAS_Fr01: 四轮驱动转速指令 (RPM)
  0x211 LAS_Fr02: 四轮转向角度指令 (deg)
  0x212 LAS_Fr03: 任务状态帧

RDM → 工控机 (Rx):
  0x1F  RDM_Fr03: 整车状态（高压就绪、急停、驻车锁等）
  0x19  RDM_Fr35: 四轮转速反馈 (RPM)
  0x1B  RDM_Fr36: 四轮转向角反馈 (deg)
  0x2A  RDM_Fr37: FL/FR 电机位置 (32bit signed)
  0x2B  RDM_Fr38: RL/RR 电机位置 (32bit signed)
"""

import os
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from builtin_interfaces.msg import Time
from robot_msgs.msg import MotorCmd, MotorState, CanFrame

import can
import cantools


# CAN ID 常量
ID_WHL_SPD_CMD  = 0x210   # 四轮转速指令
ID_WHL_YAW_CMD  = 0x211   # 四轮转向角指令
ID_STATUS_CMD   = 0x212   # 任务状态帧
ID_WHL_SPD_FBK  = 0x19    # 四轮转速反馈
ID_WHL_YAW_FBK  = 0x1B    # 四轮转向角反馈
ID_VEH_STAT_FBK = 0x1F    # 整车状态反馈
ID_FL_MOTOR_POS = 0x2A    # FL/FR 电机位置
ID_RL_MOTOR_POS = 0x2B    # RL/RR 电机位置

# 单位换算
RAD2DEG   = 180.0 / math.pi
RPM2RADS  = 2.0 * math.pi / 60.0
RADS2RPM  = 60.0 / (2.0 * math.pi)

# DBC 路径（V1.0）
DBC_PATH = '/home/admin123/Development/G60Pro/AutoPilot/20260325G60pro/src/robot_can/config/can1/G60_CAN1_RDM_V1.0.dbc'


class CanNode(Node):
    """CAN 通信节点 — 基于 DBC V1.0 协议"""

    def __init__(self):
        super().__init__('robot_can_node')

        # 参数
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('use_sim', False)

        self.can_channel = self.get_parameter('can_channel').value
        self.use_sim = self.get_parameter('use_sim').value

        # 加载 DBC
        self._db = None
        self._load_dbc()

        # CAN 总线
        self.bus = None
        self._init_can()

        # 订阅 motor_cmd
        self.motor_cmd_sub = self.create_subscription(
            MotorCmd, 'motor_cmd', self._on_motor_cmd, QoSProfile(depth=10))

        # 发布 motor_state
        self.motor_state_pub = self.create_publisher(
            MotorState, 'motor_state', QoSProfile(depth=10))

        # 发布整车状态
        self.veh_stat_pub = self.create_publisher(
            CanFrame, 'veh_status', QoSProfile(depth=10))

        # 发布原始 CAN 帧（调试）
        self.can_frame_pub = self.create_publisher(
            CanFrame, 'can_frames', QoSProfile(depth=200))

        # 发送定时器 50ms (20Hz)
        self.timer = self.create_timer(0.05, self._send_periodic)

        # 缓存上次指令
        self._last_cmd = None
        self._rolling_counter = 0

        self.get_logger().info(
            f'CAN 节点已启动 (channel={self.can_channel}, sim={self.use_sim})')

    # ──────────────────────────────────────────────────────────
    # DBC 加载
    # ──────────────────────────────────────────────────────────

    def _load_dbc(self):
        if not os.path.exists(DBC_PATH):
            self.get_logger().warn(f'DBC 不存在: {DBC_PATH}，将使用手动编解码')
            return
        try:
            self._db = cantools.db.load_file(DBC_PATH)
            names = [m.name for m in self._db.messages]
            self.get_logger().info(f'DBC 加载成功: {DBC_PATH}')
            self.get_logger().info(f'消息列表: {names}')
        except Exception as e:
            self.get_logger().error(f'DBC 加载失败: {e}')
            self._db = None

    # ──────────────────────────────────────────────────────────
    # CAN 初始化
    # ──────────────────────────────────────────────────────────

    def _init_can(self):
        if self.use_sim:
            self.get_logger().info('模拟模式，跳过 CAN 初始化')
            return
        try:
            self.bus = can.interface.Bus(channel=self.can_channel, bustype='socketcan')
            self.get_logger().info(f'CAN 已连接: {self.can_channel}')
        except Exception as e:
            self.get_logger().error(f'CAN 连接失败: {e}')
            self.bus = None

    # ──────────────────────────────────────────────────────────
    # 单位转换
    # ──────────────────────────────────────────────────────────

    @staticmethod
    def _rads_to_rpm(rads: float) -> float:
        return rads * RADS2RPM

    @staticmethod
    def _rpm_to_rads(rpm: float) -> float:
        return rpm * RPM2RADS

    @staticmethod
    def _rad_to_deg(rad: float) -> float:
        return rad * RAD2DEG

    @staticmethod
    def _deg_to_rad(deg: float) -> float:
        return deg / RAD2DEG

    # ──────────────────────────────────────────────────────────
    # 回调
    # ──────────────────────────────────────────────────────────

    def _on_motor_cmd(self, msg: MotorCmd):
        """缓存 MotorCmd 用于周期性发送"""
        self._last_cmd = msg

    def _send_periodic(self):
        """每 50ms 发送控制指令"""
        if self._last_cmd is None:
            return

        cmd = self._last_cmd
        rc = self._rolling_counter & 0xF
        self._rolling_counter = (self._rolling_counter + 1) & 0xF

        try:
            self._send_whl_spd(cmd)
            self._send_whl_yaw(cmd)
            self._send_status(rc)
        except Exception as e:
            self.get_logger().error(f'CAN 发送异常: {e}')

    # ──────────────────────────────────────────────────────────
    # 发送
    # ──────────────────────────────────────────────────────────

    def _send_whl_spd(self, cmd: MotorCmd):
        """0x210 四轮转速指令 (RPM)"""
        fl = self._rads_to_rpm(cmd.drive_velocity[0])
        fr = self._rads_to_rpm(cmd.drive_velocity[1])
        rl = self._rads_to_rpm(cmd.drive_velocity[2])
        rr = self._rads_to_rpm(cmd.drive_velocity[3])

        if self._db is not None:
            msg_def = self._db.get_message_by_name('LAS_Fr01_0x210')
            data = msg_def.encode({
                'LAS_Fr01_FLWhlSpdReq': fl,
                'LAS_Fr01_FRWhlSpdReq': fr,
                'LAS_Fr01_RLWhlSpdReq': rl,
                'LAS_Fr01_RRWhlSpdReq': rr,
            })
        else:
            import struct
            raw = [int(v * 100 + 300 + 0.5) for v in [fl, fr, rl, rr]]
            data = struct.pack('<hhhh', *raw)

        self._can_send(ID_WHL_SPD_CMD, data)

    def _send_whl_yaw(self, cmd: MotorCmd):
        """0x211 四轮转向角指令 (deg)"""
        fl = self._rad_to_deg(cmd.steer_angle[0])
        fr = self._rad_to_deg(cmd.steer_angle[1])
        rl = self._rad_to_deg(cmd.steer_angle[2])
        rr = self._rad_to_deg(cmd.steer_angle[3])

        if self._db is not None:
            msg_def = self._db.get_message_by_name('LAS_Fr02_0x211')
            data = msg_def.encode({
                'LAS_Fr02_FLWhlYawReq': fl,
                'LAS_Fr02_FRWhlYawReq': fr,
                'LAS_Fr02_RLWhlYawReq': rl,
                'LAS_Fr02_RRWhlYawReq': rr,
            })
        else:
            import struct
            raw = [int(v * 100 + 300 + 0.5) for v in [fl, fr, rl, rr]]
            data = struct.pack('<hhhh', *raw)

        self._can_send(ID_WHL_YAW_CMD, data)

    def _send_status(self, rc: int):
        """0x212 任务状态帧"""
        if self._db is not None:
            msg_def = self._db.get_message_by_name('LAS_Fr03_0x212')
            data = msg_def.encode({
                'LAS_Fr03_TskTypFeb': 0,
                'LAS_Fr03_WorkState': 0,
                'LAS_Fr03_VehStopReq': 0,
                'LAS_Fr03_VehDrvState': 0,
                'LAS_Fr03_FaultCode': 0,
                'LAS_Fr03_RollgCntr': rc,
                'LAS_Fr03_AdmNotRespondTsk': 0,
                'LAS_Fr03_F2dLidarFail': 0,
                'LAS_Fr03_B2dLidarFail': 0,
                'LAS_Fr03_F3dLidarFail': 0,
                'LAS_Fr03_LCameraFail': 0,
                'LAS_Fr03_RCameraFail': 0,
                'LAS_Fr03_FCameraFail': 0,
                'LAS_Fr03_BCameraFail': 0,
                'LAS_Fr03_ImuFail': 0,
                'LAS_Fr03_PerceptionFail': 0,
                'LAS_Fr03_PncFail': 0,
                'LAS_Fr03_LocationLost': 0,
                'LAS_Fr03_TargtPoint': 0,
                'LAS_Fr03_StopByObstacle': 0,
                'LAS_Fr03_OutOfStation': 0,
            })
        else:
            import struct
            data = struct.pack('<BBBBBBBB', 0, 0, 0, 0, 0, rc, 0, 0)

        self._can_send(ID_STATUS_CMD, data)

    def _can_send(self, can_id: int, data: bytes):
        if self.bus is None:
            return
        msg = can.Message(arbitration_id=can_id, data=data,
                          is_extended_id=False, is_fd=False, dlc=len(data))
        try:
            self.bus.send(msg)
        except Exception as e:
            self.get_logger().error(f'发送 CAN 帧 0x{can_id:03X} 失败: {e}')

    # ──────────────────────────────────────────────────────────
    # 接收
    # ──────────────────────────────────────────────────────────

    def _can_receive_callback(self):
        """在主循环中调用，接收所有 CAN 帧"""
        if self.bus is None:
            return
        try:
            while True:
                msg = self.bus.recv(timeout=0.001)
                if msg is None:
                    break
                self._handle_frame(msg)
        except Exception as e:
            self.get_logger().error(f'CAN 接收错误: {e}')

    def _handle_frame(self, msg: can.Message):
        """根据 CAN ID 分发"""
        arb = msg.arbitration_id
        self._publish_raw(msg)

        if arb == ID_WHL_SPD_FBK:
            self._decode_whl_spd(msg)
        elif arb == ID_WHL_YAW_FBK:
            self._decode_whl_yaw(msg)
        elif arb == ID_VEH_STAT_FBK:
            self._decode_veh_stat(msg)
        elif arb == ID_FL_MOTOR_POS:
            self._decode_motor_pos_2a(msg)
        elif arb == ID_RL_MOTOR_POS:
            self._decode_motor_pos_2b(msg)
        else:
            self.get_logger().debug(f'未知 CAN ID: 0x{arb:03X}')

    def _decode_whl_spd(self, msg: can.Message):
        """0x19 四轮转速反馈 RPM → MotorState.drive_velocity_feedback (rad/s)"""
        try:
            state = MotorState()
            state.stamp = self.get_clock().now().to_msg()

            if self._db is not None:
                decoded = self._db.get_message_by_name('RDM_Fr35_0x19').decode(msg.data)
                state.drive_velocity_feedback = [
                    self._rpm_to_rads(decoded.get('RDM_Fr35_FLWhlSpd', 0.0)),
                    self._rpm_to_rads(decoded.get('RDM_Fr35_FRWhlSpd', 0.0)),
                    self._rpm_to_rads(decoded.get('RDM_Fr35_RLWhlSpd', 0.0)),
                    self._rpm_to_rads(decoded.get('RDM_Fr35_RRWhlSpd', 0.0)),
                ]
            else:
                import struct
                vals = struct.unpack('<hhhh', msg.data)
                state.drive_velocity_feedback = [
                    self._rpm_to_rads(v / 100.0 - 300.0) for v in vals
                ]

            state.steer_angle_feedback = [0.0] * 4
            state.temperature = [0.0] * 4
            state.current = [0.0] * 4
            state.voltage = 0.0
            state.error_code = 0

            self.motor_state_pub.publish(state)
        except Exception as e:
            self.get_logger().error(f'解析 0x19 失败: {e}')

    def _decode_whl_yaw(self, msg: can.Message):
        """0x1B 四轮转向角反馈 deg → MotorState.steer_angle_feedback (rad)"""
        try:
            state = MotorState()
            state.stamp = self.get_clock().now().to_msg()

            if self._db is not None:
                decoded = self._db.get_message_by_name('RDM_Fr36_0x1B').decode(msg.data)
                state.steer_angle_feedback = [
                    self._deg_to_rad(decoded.get('RDM_Fr36_FLWhlYaw', 0.0)),
                    self._deg_to_rad(decoded.get('RDM_Fr36_FRWhlYaw', 0.0)),
                    self._deg_to_rad(decoded.get('RDM_Fr36_RLWhlYaw', 0.0)),
                    self._deg_to_rad(decoded.get('RDM_Fr36_RRWhlYaw', 0.0)),
                ]
            else:
                import struct
                vals = struct.unpack('<hhhh', msg.data)
                state.steer_angle_feedback = [
                    self._deg_to_rad(v / 10.0 - 200.0) for v in vals
                ]

            state.drive_velocity_feedback = [0.0] * 4
            state.temperature = [0.0] * 4
            state.current = [0.0] * 4
            state.voltage = 0.0
            state.error_code = 0

            self.motor_state_pub.publish(state)
        except Exception as e:
            self.get_logger().error(f'解析 0x1B 失败: {e}')

    def _decode_veh_stat(self, msg: can.Message):
        """0x1F 整车状态反馈"""
        try:
            if self._db is not None:
                d = self._db.get_message_by_name('RDM_Fr03_0x1F').decode(msg.data)
                self.get_logger().debug(
                    f'0x1F: HV_Ready={d.get("RDM_Fr03_EssHvReady")} '
                    f'EptRdy={d.get("RDM_Fr03_EptRdy")} '
                    f'VehStop={d.get("RDM_Fr03_VehStopReq")} '
                    f'PLock={d.get("RDM_Fr03_PLockStatReq")}')
        except Exception as e:
            self.get_logger().error(f'解析 0x1F 失败: {e}')

    def _decode_motor_pos_2a(self, msg: can.Message):
        """0x2A FL/FR 电机位置"""
        try:
            import struct
            fl, fr = struct.unpack('<ii', msg.data[0:8])
            self.get_logger().debug(f'0x2A: FL={fl}, FR={fr}')
        except Exception as e:
            self.get_logger().error(f'解析 0x2A 失败: {e}')

    def _decode_motor_pos_2b(self, msg: can.Message):
        """0x2B RL/RR 电机位置"""
        try:
            import struct
            rl, rr = struct.unpack('<ii', msg.data[0:8])
            self.get_logger().debug(f'0x2B: RL={rl}, RR={rr}')
        except Exception as e:
            self.get_logger().error(f'解析 0x2B 失败: {e}')

    def _publish_raw(self, msg: can.Message):
        """发布原始 CAN 帧"""
        frame = CanFrame()
        frame.can_id = msg.arbitration_id
        frame.dlc = msg.dlc
        frame.data = list(msg.data)
        frame.stamp = self.get_clock().now().to_msg()
        self.can_frame_pub.publish(frame)

    def destroy_node(self):
        if self.bus:
            self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanNode()

    def receive_loop():
        while rclpy.ok():
            node._can_receive_callback()

    recv_thread = threading.Thread(target=receive_loop, daemon=True)
    recv_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
