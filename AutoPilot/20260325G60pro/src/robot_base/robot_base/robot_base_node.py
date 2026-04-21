#!/usr/bin/env python3
"""
G60Pro 4轮8驱底盘运动学解算节点
订阅 /cmd_vel，输出 8 电机指令 (4转向 + 4驱动)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_msgs.msg import MotorCmd

import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class RobotBaseNode(Node):
    """4轮8驱底盘运动学解算节点"""

    def __init__(self):
        super().__init__('robot_base_node')

        # 声明参数
        self.declare_parameter('wheel_radius', 0.15)        # 轮子半径 (m)
        self.declare_parameter('wheelbase_x', 0.61)         # 前后轮距之半 (m)
        self.declare_parameter('wheelbase_y', 0.2705)      # 左右轮距之半 (m)
        self.declare_parameter('max_velocity', 1.0)          # 最大线速度 (m/s)
        self.declare_parameter('max_angular', 1.0)          # 最大角速度 (rad/s)
        self.declare_parameter('use_sim', False)            # 模拟模式
        self.declare_parameter('publish_tf', True)           # 发布 odom→base_footprint TF
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # 获取参数值
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.Lx = self.get_parameter('wheelbase_x').value   # 前后轮距之半
        self.Ly = self.get_parameter('wheelbase_y').value   # 左右轮距之半
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_angular = self.get_parameter('max_angular').value
        self.use_sim = self.get_parameter('use_sim').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # 轮子配置 [x, y] 位置 (相对于车身中心)
        # FL: 左前, FR: 右前, RL: 左后, RR: 右后
        self.wheel_positions = [
            [self.Lx, self.Ly],   # FL (左前)
            [self.Lx, -self.Ly],  # FR (右前)
            [-self.Lx, self.Ly],  # RL (左后)
            [-self.Lx, -self.Ly]   # RR (右后)
        ]

        # 里程计状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 速度积分
        self.last_time = self.get_clock().now()

        # 订阅 /cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self._cmd_vel_callback,
            QoSProfile(depth=10)
        )

        # 发布 /motor_cmd
        self.motor_cmd_pub = self.create_publisher(
            MotorCmd,
            'motor_cmd',
            QoSProfile(depth=10)
        )

        # 发布 /odom
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            QoSProfile(depth=10)
        )

        # TF 广播器：odom -> base_footprint
        self.tf_broadcaster = TransformBroadcaster(self)

        # 实车模式：10Hz 定时发布 odom，保证 AMCL 始终能找到 odom→base_footprint TF
        if not self.use_sim:
            self.create_timer(0.1, self._publish_odometry)

        self.get_logger().info('Robot Base 节点已启动')

    def _cmd_vel_callback(self, msg: Twist):
        """处理速度指令，计算 8 电机指令"""
        vx = msg.linear.x   # 前后方向 (m/s)
        vy = msg.linear.y   # 左右方向 (m/s)
        omega = msg.angular.z  # 旋转 (rad/s)

        # 限制速度
        vx = self._clamp(vx, -self.max_velocity, self.max_velocity)
        vy = self._clamp(vy, -self.max_velocity, self.max_velocity)
        omega = self._clamp(omega, -self.max_angular, self.max_angular)

        # 计算每个轮子的转向角度和转速
        steer_angles = []
        drive_velocities = []

        for wheel_pos in self.wheel_positions:
            xi = wheel_pos[0]
            yi = wheel_pos[1]

            # 计算该轮子的目标速度分量
            # 运动学：v_wheel = v_body + omega x r
            v_wheel_x = vx - omega * yi
            v_wheel_y = vy + omega * xi

            # 计算转向角度 (相对于车身纵向)
            steer_angle = math.atan2(v_wheel_y, v_wheel_x)
            steer_angles.append(steer_angle)

            # 计算驱动速度 (rad/s)
            wheel_speed = math.sqrt(v_wheel_x**2 + v_wheel_y**2)
            drive_velocity = wheel_speed / self.wheel_radius
            drive_velocities.append(drive_velocity)

        # 发布 motor_cmd
        motor_cmd = MotorCmd()
        motor_cmd.steer_angle = steer_angles
        motor_cmd.drive_velocity = drive_velocities
        motor_cmd.stamp = self.get_clock().now().to_msg()

        self.motor_cmd_pub.publish(motor_cmd)

        # 仿真模式下 Gazebo 已发布 odom 和 odom→base_footprint TF，跳过以避免冲突
        # 实车模式下更新位姿状态（odom 由定时器发布）
        if not self.use_sim:
            self._update_odometry(vx, vy, omega)

    def _update_odometry(self, vx, vy, omega):
        """更新里程计"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt > 0:
            # 更新位置 (简单积分)
            self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
            self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
            self.theta += omega * dt

            # 处理角度循环
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            self.last_time = current_time

            # 发布 odom
            self._publish_odometry()

    def _publish_odometry(self):
        """发布里程计消息和 TF"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = [0.0, 0.0, math.sin(self.theta / 2.0), math.cos(self.theta / 2.0)]
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # 设置协方差
        odom.pose.covariance = [0.0] * 36

        self.odom_pub.publish(odom)

        # 广播 TF: odom -> base_footprint
        # 注意：实车使用 Cartographer 纯定位时（map→base_footprint），
        # robot_base_node 不再发布 odom→base_footprint，避免 base_footprint 有两个父节点
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = odom.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = q[0]
            tf.transform.rotation.y = q[1]
            tf.transform.rotation.z = q[2]
            tf.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(tf)

    def _clamp(self, value, min_val, max_val):
        """限制数值范围"""
        return max(min_val, min(max_val, value))


def main(args=None):
    rclpy.init(args=args)
    node = RobotBaseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()