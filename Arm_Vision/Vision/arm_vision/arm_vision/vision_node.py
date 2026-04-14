#!/usr/bin/env python3
"""
粗位姿估算 ROS2 节点

订阅 Orbbec 相机（pyorbbecsdk Pipeline），运行 YOLO 检测充电口，
通过 RANSAC 平面拟合估算 6DoF 位姿，发布 /arm/coarse_pose 话题。

用法：
    ros2 run arm_vision vision_node

话题：
    发布  /arm/coarse_pose    geometry_msgs/PoseStamped  粗位姿（相机坐标系）
         /arm/coarse_debug    std_msgs/Float64MultiArray  [x,y,z,rx,ry,rz,conf] 调试信息
"""

import os
import sys
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

from ultralytics import YOLO
from pyorbbecsdk import Pipeline, Config, OBSensorType, OBAlignMode, FrameSet

from .utils import (
    frame_to_bgr,
    get_depth_mm,
    backproject,
    fit_plane_ransac,
    bbox_to_pointcloud,
    build_coarse_pose,
    rotation_matrix_to_euler,
    rotation_matrix_to_quaternion,
)


def get_vision_root():
    """获取 Vision 根目录路径（arm_vision 的上一级）。"""
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class VisionNode(Node):
    """粗位姿估算节点"""

    def __init__(self):
        super().__init__('vision_node')

        # 参数声明
        self.declare_parameter('yolo_model', 'detect/dc_charging_v2i/weights/best.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('ransac_iterations', 100)
        self.declare_parameter('ransac_threshold_m', 0.005)
        self.declare_parameter('publish_rate', 10.0)

        model_path = self.get_parameter('yolo_model').value
        self.confidence = self.get_parameter('confidence').value
        self.ransac_iterations = self.get_parameter('ransac_iterations').value
        self.ransac_threshold_m = self.get_parameter('ransac_threshold_m').value
        publish_rate = self.get_parameter('publish_rate').value

        # 路径：相对于 Vision 根目录
        vision_root = get_vision_root()
        if not os.path.isabs(model_path):
            model_path = os.path.join(vision_root, model_path)

        # 发布者
        self.pose_pub = self.create_publisher(PoseStamped, '/arm/coarse_pose', 10)
        self.debug_pub = self.create_publisher(Float64MultiArray, '/arm/coarse_debug', 10)

        # 加载 YOLO 模型
        self.get_logger().info(f'加载 YOLO 模型: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('YOLO 模型加载完成')

        # 相机内参（从 Pipeline 获取）
        self.fx = None
        self.fy = None
        self.cx_i = None
        self.cy_i = None
        self.camera_ready = False

        # ROS2 定时器用于发布（从 pipeline 线程同步获取最新帧）
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # 最新检测结果（线程安全）
        self.latest_pose = None
        self.latest_debug = None
        self.lock = threading.Lock()

        # 启动相机线程
        self.running = True
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.camera_thread.start()

        self.get_logger().info('vision_node 粗位姿节点已启动')

    def _camera_loop(self):
        """相机采集线程：持续读取帧 → YOLO检测 → 发布结果"""
        try:
            pipeline = Pipeline()
            config = Config()

            color_profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            color_profile = color_profiles.get_default_video_stream_profile()
            config.enable_stream(color_profile)

            depth_profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            depth_profile = depth_profiles.get_default_video_stream_profile()
            config.enable_stream(depth_profile)

            config.set_align_mode(OBAlignMode.HW_MODE)
            pipeline.start(config)
            self.get_logger().info('相机 Pipeline 已启动')

            # 获取内参
            cam_param = pipeline.get_camera_param()
            self.fx = cam_param.rgb_intrinsic.fx
            self.fy = cam_param.rgb_intrinsic.fy
            self.cx_i = cam_param.rgb_intrinsic.cx
            self.cy_i = cam_param.rgb_intrinsic.cy
            self.camera_ready = True
            self.get_logger().info(
                f'相机内参: fx={self.fx:.2f} fy={self.fy:.2f} cx={self.cx_i:.2f} cy={self.cy_i:.2f}')

            while self.running:
                frames: FrameSet = pipeline.wait_for_frames(100)
                if frames is None:
                    continue

                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if color_frame is None or depth_frame is None:
                    continue

                color_image = frame_to_bgr(color_frame)
                if color_image is None:
                    continue

                d_w = depth_frame.get_width()
                d_h = depth_frame.get_height()
                scale = depth_frame.get_depth_scale()
                depth_raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((d_h, d_w))
                depth_mm = depth_raw.astype(np.float32) * scale

                ch, cw = color_image.shape[:2]

                # YOLO 推理
                results = self.model(color_image, conf=self.confidence, verbose=False)
                r = results[0]

                if r.boxes is None or len(r.boxes) == 0:
                    with self.lock:
                        self.latest_pose = None
                        self.latest_debug = None
                    continue

                names = r.names
                xyxy = r.boxes.xyxy.cpu().numpy().astype(int)
                cls = r.boxes.cls.cpu().numpy().astype(int)
                conf = r.boxes.conf.cpu().numpy()

                # 找 charging_port
                port_boxes = []
                for box, c, p in zip(xyxy, cls, conf):
                    label = names.get(c, str(c)) if isinstance(names, dict) else names[c]
                    if label == 'charging_port':
                        port_boxes.append((box, p))

                if not port_boxes:
                    with self.lock:
                        self.latest_pose = None
                        self.latest_debug = None
                    continue

                # 取置信度最高的 charging_port
                port_box = sorted(port_boxes, key=lambda x: x[1], reverse=True)[0][0]
                px1, py1, px2, py2 = port_box

                # bbox 中心点深度
                uc = (px1 + px2) // 2
                vc = (py1 + py2) // 2
                dpx1 = int(px1 * d_w / cw)
                dpx2 = int(px2 * d_w / cw)
                dpy1 = int(py1 * d_h / ch)
                dpy2 = int(py2 * d_h / ch)

                patch = depth_mm[dpy1:dpy2, dpx1:dpx2]
                valid = patch[(patch > 0) & np.isfinite(patch)]
                z_mm = float(np.median(valid)) if valid.size > 0 else None

                if z_mm is None:
                    with self.lock:
                        self.latest_pose = None
                        self.latest_debug = None
                    continue

                center_3d = backproject(uc, vc, z_mm, self.fx, self.fy, self.cx_i, self.cy_i)

                # RANSAC 平面拟合
                pts3d = bbox_to_pointcloud(depth_mm, (dpx1, dpy1, dpx2, dpy2),
                                           self.fx, self.fy, self.cx_i, self.cy_i)
                normal = fit_plane_ransac(pts3d, n_iter=self.ransac_iterations,
                                          thresh_m=self.ransac_threshold_m)

                if normal is None:
                    normal = np.array([0.0, 0.0, -1.0])

                # 构建粗位姿
                T_coarse = build_coarse_pose(center_3d, normal)

                t = T_coarse[:3, 3]
                R = T_coarse[:3, :3]
                rx, ry, rz = rotation_matrix_to_euler(R)
                qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

                # 置信度（取 YOLO 置信度）
                best_conf = sorted(port_boxes, key=lambda x: x[1], reverse=True)[0][1]

                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'camera_link'
                pose_msg.pose.position.x = float(t[0])
                pose_msg.pose.position.y = float(t[1])
                pose_msg.pose.position.z = float(t[2])
                pose_msg.pose.orientation.x = float(qx)
                pose_msg.pose.orientation.y = float(qy)
                pose_msg.pose.orientation.z = float(qz)
                pose_msg.pose.orientation.w = float(qw)

                debug_msg = Float64MultiArray()
                debug_msg.data = [
                    float(t[0]), float(t[1]), float(t[2]),
                    float(rx), float(ry), float(rz),
                    float(best_conf)
                ]

                with self.lock:
                    self.latest_pose = pose_msg
                    self.latest_debug = debug_msg

        except Exception as e:
            self.get_logger().error(f'相机线程异常: {e}')
            import traceback
            traceback.print_exc()

    def timer_callback(self):
        """定时发布最新检测结果"""
        with self.lock:
            pose = self.latest_pose
            debug = self.latest_debug

        if pose is not None:
            self.pose_pub.publish(pose)
        if debug is not None:
            self.debug_pub.publish(debug)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
