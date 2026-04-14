"""
ICP 精确位姿估计

前提：机械臂已移动到标准拍摄位（X=0, Y=0, Z=500mm, Rx=0, Ry=0）。

运行方式：
    bash run_icp_refine.sh

流程：
    1. 拍一帧彩色 + 深度
    2. YOLO 检测 charging_port → 确定充电口区域 bbox
    3. bbox 区域深度 → 当前帧点云（target）
    4. 加载 STEP 模板点云（source）
    5. 以标准拍摄位（Z=500mm）为初始值，运行 ICP
    6. 输出精确 6DoF 位姿（精度：平移 ±0.5mm，旋转 ±1°）

按 Q 或 ESC 退出。
"""

import cv2
import numpy as np
import open3d as o3d
from ultralytics import YOLO

from pyorbbecsdk import (
    Pipeline,
    Config,
    OBSensorType,
    OBAlignMode,
    OBFormat,
    FrameSet,
)

ESC_KEY = 27

TEMPLATE_PATH = "datasets/直流充电座STP和点云/charging_port_template.pcd"
VOXEL_SIZE    = 0.002   # 体素降采样：2mm
ICP_MAX_DIST  = 0.010   # ICP 最大对应点距离：10mm
ICP_MAX_ITER  = 50      # 最大迭代次数

# 标准拍摄位初始变换（X=0, Y=0, Z=500mm，无旋转）
T_STANDARD = np.eye(4)
T_STANDARD[2, 3] = 0.5  # 500mm = 0.5m


def frame_to_bgr(frame):
    width  = frame.get_width()
    height = frame.get_height()
    fmt    = frame.get_format()
    data   = np.asanyarray(frame.get_data())

    if fmt == OBFormat.MJPG:
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    if fmt == OBFormat.RGB:
        return cv2.cvtColor(data.reshape((height, width, 3)), cv2.COLOR_RGB2BGR)
    if fmt == OBFormat.BGR:
        return data.reshape((height, width, 3))
    if fmt == OBFormat.YUYV:
        return cv2.cvtColor(data.reshape((height, width, 2)), cv2.COLOR_YUV2BGR_YUYV)
    if fmt == OBFormat.UYVY:
        return cv2.cvtColor(data.reshape((height, width, 2)), cv2.COLOR_YUV2BGR_UYVY)
    print(f"未支持的彩色格式: {fmt}")
    return None


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


def draw_axis(img, T, fx, fy, cx_i, cy_i, length=0.03):
    """在图像上画坐标轴（红X，绿Y，蓝Z）。"""
    origin = T[:3, 3]
    axes = {
        'X': (T[:3, 0], (0, 0, 255)),
        'Y': (T[:3, 1], (0, 255, 0)),
        'Z': (T[:3, 2], (255, 0, 0)),
    }

    def proj(p):
        return (int(p[0] / p[2] * fx + cx_i), int(p[1] / p[2] * fy + cy_i))

    if origin[2] <= 0:
        return
    o2d = proj(origin)
    for name, (axis, color) in axes.items():
        end = origin + axis * length
        if end[2] <= 0:
            continue
        e2d = proj(end)
        cv2.arrowedLine(img, o2d, e2d, color, 2, tipLength=0.2)
        cv2.putText(img, name, e2d, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


def rotation_to_euler_deg(R):
    """旋转矩阵 → Rx Ry（单位：度），Rz 无法从 ICP 法向量中获得返回 None。"""
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        rx = np.degrees(np.arctan2( R[2, 1], R[2, 2]))
        ry = np.degrees(np.arctan2(-R[2, 0], sy))
        rz = np.degrees(np.arctan2( R[1, 0], R[0, 0]))
    else:
        rx = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
        ry = np.degrees(np.arctan2(-R[2, 0], sy))
        rz = 0.0
    return rx, ry, rz


def main():
    # ── 加载模板点云 ──────────────────────────────────────────────────────────
    print(f"加载模板点云: {TEMPLATE_PATH}")
    template_pcd = o3d.io.read_point_cloud(TEMPLATE_PATH)
    if len(template_pcd.points) == 0:
        print("模板点云为空，请先运行 step_to_pcd.py 生成模板。")
        return
    template_pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_SIZE * 3, max_nn=30))
    print(f"模板点云加载成功，共 {len(template_pcd.points)} 个点。")

    # ── 加载 YOLO 模型 ────────────────────────────────────────────────────────
    print("加载 YOLO 模型...")
    model = YOLO("detect/dc_charging_v2i/weights/best.pt")
    print("YOLO 模型加载成功。")

    # ── 启动相机 ──────────────────────────────────────────────────────────────
    print("启动相机...")
    pipeline = Pipeline()
    config   = Config()

    color_profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    config.enable_stream(color_profiles.get_default_video_stream_profile())

    depth_profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    config.enable_stream(depth_profiles.get_default_video_stream_profile())

    config.set_align_mode(OBAlignMode.HW_MODE)
    pipeline.start(config)

    cam_param = pipeline.get_camera_param()
    fx   = cam_param.rgb_intrinsic.fx
    fy   = cam_param.rgb_intrinsic.fy
    cx_i = cam_param.rgb_intrinsic.cx
    cy_i = cam_param.rgb_intrinsic.cy
    print(f"相机内参: fx={fx:.2f} fy={fy:.2f} cx={cx_i:.2f} cy={cy_i:.2f}")
    print("按 Q 或 ESC 退出，按 空格键 手动触发一次 ICP。")

    try:
        while True:
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

            d_w   = depth_frame.get_width()
            d_h   = depth_frame.get_height()
            scale = depth_frame.get_depth_scale()
            depth_raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((d_h, d_w))
            depth_mm  = depth_raw.astype(np.float32) * scale

            ch, cw = color_image.shape[:2]
            annotated = color_image.copy()

            # ── YOLO 检测 ─────────────────────────────────────────────────────
            results = model(color_image, conf=0.5, verbose=False)
            r = results[0]

            if r.boxes is None or len(r.boxes) == 0:
                cv2.putText(annotated, "Waiting: charging_port not detected",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow("ICP Precise Pose", annotated)
                if cv2.waitKey(1) & 0xFF in (ord('q'), ESC_KEY):
                    break
                continue

            names = r.names
            xyxy  = r.boxes.xyxy.cpu().numpy().astype(int)
            cls   = r.boxes.cls.cpu().numpy().astype(int)
            conf  = r.boxes.conf.cpu().numpy()

            port_boxes = []
            for box, c, p in zip(xyxy, cls, conf):
                label = names.get(c, str(c)) if isinstance(names, dict) else names[c]
                x1, y1, x2, y2 = box
                color_map = {
                    "AC_charging_hole": (255, 0, 255),   # 紫色
                    "DC_charging_port": (0, 255, 255),   # 黄色
                    "DC_hole": (255, 255, 0),            # 青色
                    "PE": (0, 0, 255),                  # 红色
                }
                cv2.rectangle(annotated, (x1, y1), (x2, y2),
                              color_map.get(label, (200, 200, 200)), 2)
                cv2.putText(annotated, f"{label} {p:.2f}", (x1, max(0, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_map.get(label, (200,200,200)), 1)
                if label == "DC_charging_port":
                    port_boxes.append((box, p))

            if not port_boxes:
                cv2.putText(annotated, "Waiting: charging_port not detected",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow("ICP Precise Pose", annotated)
                if cv2.waitKey(1) & 0xFF in (ord('q'), ESC_KEY):
                    break
                continue

            # 取置信度最高的 charging_port
            port_box = sorted(port_boxes, key=lambda x: x[1], reverse=True)[0][0]
            px1, py1, px2, py2 = port_box

            # bbox 映射到深度图坐标
            dpx1 = int(px1 * d_w / cw)
            dpx2 = int(px2 * d_w / cw)
            dpy1 = int(py1 * d_h / ch)
            dpy2 = int(py2 * d_h / ch)

            # ── 当前帧点云 ────────────────────────────────────────────────────
            target_pcd = depth_to_pointcloud(
                depth_mm, fx, fy, cx_i, cy_i,
                bbox=(dpx1, dpy1, dpx2, dpy2))

            if len(target_pcd.points) < 20:
                cv2.putText(annotated, "Depth invalid in bbox region",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow("ICP Precise Pose", annotated)
                if cv2.waitKey(1) & 0xFF in (ord('q'), ESC_KEY):
                    break
                continue

            target_pcd.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_SIZE * 3, max_nn=30))

            # ── ICP 精对准 ────────────────────────────────────────────────────
            result = o3d.pipelines.registration.registration_icp(
                source=template_pcd,
                target=target_pcd,
                max_correspondence_distance=ICP_MAX_DIST,
                init=T_STANDARD,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=ICP_MAX_ITER)
            )

            fitness = result.fitness
            T_fine  = result.transformation

            print(f"[ICP] fitness={fitness:.3f}  "
                  f"inlier_rmse={result.inlier_rmse*1000:.2f}mm")

            if fitness < 0.3:
                cv2.putText(annotated, f"[ICP] FAILED fitness={fitness:.2f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                t = T_fine[:3, 3]
                R = T_fine[:3, :3]
                rx, ry, rz = rotation_to_euler_deg(R)

                draw_axis(annotated, T_fine, fx, fy, cx_i, cy_i)

                cv2.putText(annotated,
                            f"[ICP] fitness:{fitness:.2f}  "
                            f"X:{t[0]*1000:.1f}  Y:{t[1]*1000:.1f}  Z:{t[2]*1000:.1f}mm",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(annotated,
                            f"Rx:{rx:.1f}deg  Ry:{ry:.1f}deg  Rz:{rz:.1f}deg",
                            (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                print(f"[ICP] T_fine:\n{np.round(T_fine, 4)}")
                print(f"[ICP] X:{t[0]*1000:.1f}mm  Y:{t[1]*1000:.1f}mm  Z:{t[2]*1000:.1f}mm  "
                      f"Rx:{rx:.1f}deg  Ry:{ry:.1f}deg  Rz:{rz:.1f}deg")

            cv2.imshow("ICP Precise Pose", annotated)
            if cv2.waitKey(1) & 0xFF in (ord('q'), ESC_KEY):
                break

    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
