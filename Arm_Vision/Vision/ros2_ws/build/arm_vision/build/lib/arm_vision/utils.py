"""
视觉算法工具函数
从 pose_estimation.py 和 icp_refine.py 提取的核心算法，不含 ROS2/OpenCV 依赖。
"""

import numpy as np


def frame_to_bgr(frame):
    """把 Orbbec 彩色帧转成 BGR ndarray，支持 MJPG/RGB/BGR/YUYV/UYVY。"""
    from pyorbbecsdk import OBFormat

    width = frame.get_width()
    height = frame.get_height()
    fmt = frame.get_format()
    data = np.asanyarray(frame.get_data())

    if fmt == OBFormat.MJPG:
        import cv2
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    if fmt == OBFormat.RGB:
        import cv2
        return cv2.cvtColor(data.reshape((height, width, 3)), cv2.COLOR_RGB2BGR)
    if fmt == OBFormat.BGR:
        return data.reshape((height, width, 3))
    if fmt == OBFormat.YUYV:
        import cv2
        return cv2.cvtColor(data.reshape((height, width, 2)), cv2.COLOR_YUV2BGR_YUYV)
    if fmt == OBFormat.UYVY:
        import cv2
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


def rotation_matrix_to_euler(R):
    """
    旋转矩阵 → 欧拉角（ZYX顺序，单位：度）。
    与 pose_estimation.py / icp_refine.py 保持一致。
    """
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        rx = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
        ry = np.degrees(np.arctan2(-R[2, 0], sy))
        rz = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    else:
        rx = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
        ry = np.degrees(np.arctan2(-R[2, 0], sy))
        rz = 0.0

    # Rz 修正：消除 Z 轴方向定义引入的 180° 固定偏置
    if rz > 90:
        rz -= 180
    elif rz < -90:
        rz += 180

    return rx, ry, rz


def rotation_matrix_to_quaternion(R):
    """旋转矩阵 → 四元数 (x, y, z, w)。"""
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    # 归一化
    norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    return np.array([qx / norm, qy / norm, qz / norm, qw / norm])
