#!/usr/bin/env python3
"""
ICP 精定位 ROS2 节点

订阅 Orbbec 相机（pyorbbecsdk Pipeline），运行 YOLO 检测 + ICP 点云配准，
输出精确 6DoF 位姿，发布 /arm/fine_pose 话题。

用法：
    ros2 run arm_vision icp_node

话题：
    发布  /arm/fine_pose     geometry_msgs/PoseStamped  精位姿（相机坐标系）
         /arm/fine_debug    std_msgs/Float64MultiArray  [x,y,z,rx,ry,rz,fitness,inlier_rmse]
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
import open3d as o3d

from .utils import (
    frame_to_bgr,
    backproject,
    rotation_matrix_to_euler,
    rotation_matrix_to_quaternion,
)


VOXEL_SIZE = 0.002     # 体素降采样：2mm
ICP_MAX_DIST = 0.010   # ICP 最大对应点距离：10mm
ICP_MAX_ITER = 50      # 最大迭代次数


def get_vision_root():
    """获取 Vision 根目录路径。"""
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def depth_to_pointcloud(depth_mm, fx, fy, cx, cy, bbox=None):
    """bbox 区域深度图 → Open3D 点云（单位：米）。"""
    h, w = depth_mm.shape
    if bbox is not None:
        x1, y1, x2, y2 = bbox
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
    else:
        x1, y1, x2, y2 = 0, 0, w, h

    us, vs = np.meshgrid(np.arange(x1, x2), np.arange(y1, y2))
    zs = depth_mm[y1:y2, x1:x2].astype(np.float32) / 1000.0

    valid = (zs > 0.05) & (zs < 2.0) & np.isfinite(zs)
    zs = zs[valid]
    us = us[valid].astype(np.float32)
    vs = vs[valid].astype(np.float32)

    if zs.size == 0:
        return o3d.geometry.PointCloud()

    xs = (us - cx) * zs / fx
    ys = (vs - cy) * zs / fy
    pts = np.stack([xs, ys, zs], axis=1)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd = pcd.voxel_down_sample(VOXEL_SIZE)
    return pcd


class IcpNode(Node):
    """ICP 精定位节点"""

    def __init__(self):
        super().__init__('icp_node')

        # 参数声明
        self.declare_parameter('yolo_model', 'detect/dc_charging_v2i/weights/best.pt')
        self.declare_parameter('template_path',
                               'datasets/直流充电座STP和点云/charging_port_template.pcd')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('voxel_size', VOXEL_SIZE)
        self.declare_parameter('icp_max_dist', ICP_MAX_DIST)
        self.declare_parameter('icp_max_iter', ICP_MAX_ITER)
        self.declare_parameter('publish_rate', 10.0)

        model_path = self.get_parameter('yolo_model').value
        template_path = self.get_parameter('template_path').value
        self.confidence = self.get_parameter('confidence').value
        self.icp_max_dist = self.get_parameter('icp_max_dist').value
        self.icp_max_iter = self.get_parameter('icp_max_iter').value
        publish_rate = self.get_parameter('publish_rate').value

        # 路径：相对于 Vision 根目录
        vision_root = get_vision_root()
        if not os.path.isabs(model_path):
            model_path = os.path.join(vision_root, model_path)
        if not os.path.isabs(template_path):
            template_path = os.path.join(vision_root, template_path)

        # 发布者
        self.pose_pub = self.create_publisher(PoseStamped, '/arm/fine_pose', 10)
        self.debug_pub = self.create_publisher(Float64MultiArray, '/arm/fine_debug', 10)

        # 加载 YOLO 模型
        self.get_logger().info(f'加载 YOLO 模型: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('YOLO 模型加载完成')

        # 加载模板点云
        self.get_logger().info(f'加载模板点云: {template_path}')
        self.template_pcd = o3d.io.read_point_cloud(template_path)
        if len(self.template_pcd.points) == 0:
            self.get_logger().error('模板点云为空，请先运行 step_to_pcd.py 生成模板')
            self.template_pcd = None
        else:
            self.template_pcd.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_SIZE * 3, max_nn=30))
            self.get_logger().info(f'模板点云加载成功，共 {len(self.template_pcd.points)} 个点')

        # 标准拍摄位初始变换（Z=500mm，无旋转）
        self.T_standard = np.eye(4)
        self.T_standard[2, 3] = 0.5

        # 相机内参
        self.fx = None
        self.fy = None
        self.cx_i = None
        self.cy_i = None
        self.camera_ready = False

        # 最新结果
        self.latest_pose = None
        self.latest_debug = None
        self.lock = threading.Lock()

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.running = True
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.camera_thread.start()

        self.get_logger().info('icp_node ICP 精定位节点已启动')

    def _camera_loop(self):
        """相机采集线程：YOLO检测 → ICP配准 → 发布结果"""
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

                # YOLO 检测
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

                port_box = sorted(port_boxes, key=lambda x: x[1], reverse=True)[0][0]
                px1, py1, px2, py2 = port_box

                # bbox 映射到深度图
                dpx1 = int(px1 * d_w / cw)
                dpx2 = int(px2 * d_w / cw)
                dpy1 = int(py1 * d_h / ch)
                dpy2 = int(py2 * d_h / ch)

                # 当前帧点云
                target_pcd = depth_to_pointcloud(
                    depth_mm, self.fx, self.fy, self.cx_i, self.cy_i,
                    bbox=(dpx1, dpy1, dpx2, dpy2))

                if len(target_pcd.points) < 20:
                    with self.lock:
                        self.latest_pose = None
                        self.latest_debug = None
                    continue

                target_pcd.estimate_normals(
                    o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_SIZE * 3, max_nn=30))

                # ICP 配准
                if self.template_pcd is None:
                    with self.lock:
                        self.latest_pose = None
                        self.latest_debug = None
                    continue

                result = o3d.pipelines.registration.registration_icp(
                    source=self.template_pcd,
                    target=target_pcd,
                    max_correspondence_distance=self.icp_max_dist,
                    init=self.T_standard,
                    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                        max_iteration=self.icp_max_iter)
                )

                fitness = result.fitness
                T_fine = result.transformation
                inlier_rmse = result.inlier_rmse

                t = T_fine[:3, 3]
                R = T_fine[:3, :3]
                rx, ry, rz = rotation_matrix_to_euler(R)
                qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

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
                    float(fitness), float(inlier_rmse * 1000)  # inlier_rmse 单位转为 mm
                ]

                with self.lock:
                    self.latest_pose = pose_msg
                    self.latest_debug = debug_msg

        except Exception as e:
            self.get_logger().error(f'相机线程异常: {e}')
            import traceback
            traceback.print_exc()

    def timer_callback(self):
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
    node = IcpNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
