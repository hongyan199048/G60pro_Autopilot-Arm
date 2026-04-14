"""
GB/T 20234.3 直流充电口 6DoF 位姿估计

粗位姿方法（方案A）：
  1. YOLO11 检测 charging_port（DC_hole / PE 作为辅助信息显示）
  2. charging_port bbox 区域内深度点云 → RANSAC 平面拟合 → 法向量（Z轴）
  3. bbox 中心点 + 深度 → 反投影得到充电座中心3D坐标（平移）
  4. Z轴（法向量）+ 世界Y轴参考 → 正交化构建完整旋转矩阵
  5. 输出粗位姿 T_coarse（4×4），供后续 ICP 精化使用

按 Q 或 ESC 退出。
"""

import cv2
import numpy as np
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

CLS_CHARGING_PORT = "charging_port"
CLS_DC_HOLE       = "DC_hole"
CLS_PE            = "PE"


def frame_to_bgr(frame):
    """把 Orbbec 彩色帧转成 BGR ndarray，支持 MJPG/RGB/BGR/YUYV/UYVY。"""
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


def get_depth_mm(depth_mm, cx, cy, half=3):
    """在 (cx,cy) 周围小窗口取有效深度中值，单位毫米。"""
    h, w = depth_mm.shape
    x0, x1 = max(0, cx - half), min(w, cx + half + 1)
    y0, y1 = max(0, cy - half), min(h, cy + half + 1)
    patch = depth_mm[y0:y1, x0:x1]
    valid = patch[(patch > 0) & np.isfinite(patch)]
    return float(np.median(valid)) if valid.size > 0 else None


def backproject(u, v, z_mm, fx, fy, cx, cy):
    """像素坐标 + 深度 → 相机坐标系3D点（单位：米）。"""
    z = z_mm / 1000.0
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.array([x, y, z])


def fit_plane_ransac(points, n_iter=100, thresh_m=0.005):
    """
    对 Nx3 点云做 RANSAC 平面拟合，返回单位法向量。
    thresh_m: 内点距离阈值（米）
    """
    if len(points) < 10:
        return None

    best_normal = None
    best_inliers = 0

    for _ in range(n_iter):
        idx = np.random.choice(len(points), 3, replace=False)
        p0, p1, p2 = points[idx]
        n = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        dists = np.abs((points - p0) @ n)
        inliers = np.sum(dists < thresh_m)
        if inliers > best_inliers:
            best_inliers = inliers
            best_normal = n

    return best_normal


def bbox_to_pointcloud(depth_mm, bbox, fx, fy, cx_i, cy_i, step=2):
    """
    将 bbox 区域内的深度像素反投影为3D点云（单位：米）。
    step: 降采样步长，加快计算。
    """
    x1, y1, x2, y2 = bbox
    pts = []
    for v in range(y1, y2, step):
        for u in range(x1, x2, step):
            z = get_depth_mm(depth_mm, u, v, half=0)
            if z is None:
                continue
            pts.append(backproject(u, v, z, fx, fy, cx_i, cy_i))
    return np.array(pts) if pts else None


def build_coarse_pose(center_3d, normal):
    """
    仅用充电座中心点 + 法向量构建粗位姿（4×4变换矩阵）。

    坐标系定义：
      Z轴：法向量（充电座朝向，即插入方向）
      X轴：Z × 世界Y轴 正交化（充电座左右方向）
      Y轴：Z × X 正交化（充电座上下方向）
      平移：充电座中心3D坐标

    返回 T_coarse (4×4) 或 None。
    """
    # Z轴：法向量朝向相机（Z分量为负表示指向相机）
    z_axis = normal.copy()
    if z_axis[2] > 0:
        z_axis = -z_axis
    z_axis = z_axis / np.linalg.norm(z_axis)

    # X轴：用世界Y轴 [0,1,0] 和 Z轴叉积，避免万向锁
    world_y = np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(world_y, z_axis)
    x_norm = np.linalg.norm(x_axis)
    if x_norm < 1e-6:
        # 特殊情况：Z轴平行于Y轴，改用X轴参考
        world_y = np.array([1.0, 0.0, 0.0])
        x_axis = np.cross(world_y, z_axis)
        x_norm = np.linalg.norm(x_axis)
    x_axis = x_axis / x_norm

    # Y轴：Z × X 正交化
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)

    T = np.eye(4)
    T[:3, 0] = x_axis
    T[:3, 1] = y_axis
    T[:3, 2] = z_axis
    T[:3, 3] = center_3d
    return T


def draw_axis(img, T, fx, fy, cx_i, cy_i, length=0.03):
    """在图像上画充电口坐标系三轴（红X，绿Y，蓝Z）。"""
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


def main():
    print("Loading YOLO model...")
    model    = YOLO("detect/dc_charging_v2i/weights/best.pt")
    print("YOLO model loaded.")
    print("Starting camera pipeline...")
    pipeline = Pipeline()
    config   = Config()

    color_profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    color_profile  = color_profiles.get_default_video_stream_profile()
    config.enable_stream(color_profile)

    depth_profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    depth_profile  = depth_profiles.get_default_video_stream_profile()
    config.enable_stream(depth_profile)

    config.set_align_mode(OBAlignMode.HW_MODE)
    pipeline.start(config)
    print("Camera pipeline started.")

    cam_param = pipeline.get_camera_param()
    fx   = cam_param.rgb_intrinsic.fx
    fy   = cam_param.rgb_intrinsic.fy
    cx_i = cam_param.rgb_intrinsic.cx
    cy_i = cam_param.rgb_intrinsic.cy
    print(f"Camera intrinsics: fx={fx:.2f} fy={fy:.2f} cx={cx_i:.2f} cy={cy_i:.2f}")

    # TODO: 替换为实际手眼标定结果
    T_cam2gripper = np.eye(4)

    print("Start: press Q or ESC to quit")

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

            # ── YOLO 推理 ────────────────────────────────────────────────────
            results = model(color_image, conf=0.5, verbose=False)
            r = results[0]

            annotated = color_image.copy()

            if r.boxes is None or len(r.boxes) == 0:
                cv2.imshow("DC Charging Port 6DoF Pose", annotated)
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
                color_map = {
                    "AC_charging_hole": (255, 0, 255),   # 紫色
                    "DC_charging_port": (0, 255, 255),   # 黄色
                    "DC_hole": (255, 255, 0),            # 青色
                    "PE": (0, 0, 255),                  # 红色
                }
                draw_color = color_map.get(label, (200, 200, 200))
                x1, y1, x2, y2 = box
                cv2.rectangle(annotated, (x1, y1), (x2, y2), draw_color, 2)
                cv2.putText(annotated, f"{label} {p:.2f}", (x1, max(0, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 1)

                if label == "DC_charging_port":
                    port_boxes.append((box, p))

            # ── 粗位姿估计（只需要 charging_port）───────────────────────────
            if not port_boxes:
                cv2.putText(annotated, "Waiting: charging_port not detected",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                # 取置信度最高的 charging_port
                port_box = sorted(port_boxes, key=lambda x: x[1], reverse=True)[0][0]
                px1, py1, px2, py2 = port_box

                # ① bbox 区域深度中值 → 3D坐标（平移，比单点更鲁棒）
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
                    cv2.putText(annotated, "Depth invalid in bbox region",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    center_3d = backproject(uc, vc, z_mm, fx, fy, cx_i, cy_i)

                    # ② bbox 区域点云 → RANSAC 平面拟合 → 法向量（Z轴）
                    pts3d = bbox_to_pointcloud(depth_mm, (dpx1, dpy1, dpx2, dpy2),
                                               fx, fy, cx_i, cy_i)
                    normal = fit_plane_ransac(pts3d) if pts3d is not None else None

                    if normal is None:
                        # 点云不足时，退回假设正对相机（Z轴朝相机）
                        normal = np.array([0.0, 0.0, -1.0])
                        cv2.putText(annotated, "RANSAC failed, using default normal",
                                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

                    # ③ 构建粗位姿
                    T_coarse = build_coarse_pose(center_3d, normal)

                    # 画坐标轴
                    draw_axis(annotated, T_coarse, fx, fy, cx_i, cy_i)

                    t = T_coarse[:3, 3]
                    R = T_coarse[:3, :3]
                    dist_mm = np.linalg.norm(t) * 1000

                    # 旋转矩阵 → 欧拉角（ZYX顺序，单位：度）
                    # 注：粗定位中 Rx/Ry 为近似值，Rz 反映充电座平面内旋转
                    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
                    if sy > 1e-6:
                        rx = np.degrees(np.arctan2( R[2, 1], R[2, 2]))
                        ry = np.degrees(np.arctan2(-R[2, 0], sy))
                        rz = np.degrees(np.arctan2( R[1, 0], R[0, 0]))
                    else:
                        rx = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
                        ry = np.degrees(np.arctan2(-R[2, 0], sy))
                        rz = 0.0

                    # Rz 修正：消除 Z 轴方向定义引入的 180° 固定偏置
                    if rz > 90:
                        rz -= 180
                    elif rz < -90:
                        rz += 180

                    cv2.putText(annotated,
                                f"[COARSE] dist:{dist_mm:.0f}mm  "
                                f"X:{t[0]*1000:.1f}  Y:{t[1]*1000:.1f}  Z:{t[2]*1000:.1f}mm",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(annotated,
                                f"Rx:{rx:.1f}deg  Ry:{ry:.1f}deg  Rz:{rz:.1f}deg",
                                (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    print(f"T_coarse:\n{np.round(T_coarse, 4)}")
                    print(f"Rx:{rx:.1f}deg  Ry:{ry:.1f}deg  Rz:{rz:.1f}deg")

            cv2.imshow("DC Charging Port 6DoF Pose", annotated)
            if cv2.waitKey(1) & 0xFF in (ord('q'), ESC_KEY):
                break

    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
