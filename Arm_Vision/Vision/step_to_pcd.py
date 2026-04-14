"""
STP → 点云(.pcd) 转换脚本

用法：
    python step_to_pcd.py

输出：
    datasets/charging_port_template.pcd   ICP 用的模板点云
    datasets/charging_port_template.ply   可视化用（可用 Open3D 查看）
"""

import numpy as np
import open3d as o3d

from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.TopoDS import topods

STEP_FILE     = "datasets/国标直流充电插座.STEP"
OUT_PCD       = "datasets/charging_port_template.pcd"
OUT_PLY       = "datasets/charging_port_template.ply"
MESH_DEFLECT  = 0.1    # 网格精度（mm），越小越精细，越慢
N_POINTS      = 8000   # 采样点数


def step_to_mesh(step_path: str, deflection: float = 0.1):
    """读取 STEP 文件，曲面离散化为三角网格，返回 (顶点数组, 面索引数组)。"""
    reader = STEPControl_Reader()
    status = reader.ReadFile(step_path)
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP 文件读取失败: {step_path}")
    reader.TransferRoots()
    shape = reader.OneShape()

    # 曲面离散化（deflection 单位与模型单位一致，通常为 mm）
    mesh = BRepMesh_IncrementalMesh(shape, deflection, False, 0.5, True)
    mesh.Perform()

    vertices = []
    triangles = []
    offset = 0

    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = topods.Face(explorer.Current())
        location = face.Location()
        triangulation = BRep_Tool.Triangulation(face, location)

        if triangulation is not None:
            # 顶点
            for i in range(1, triangulation.NbNodes() + 1):
                node = triangulation.Node(i)
                vertices.append([node.X(), node.Y(), node.Z()])

            # 三角面片
            for i in range(1, triangulation.NbTriangles() + 1):
                tri = triangulation.Triangle(i)
                n1, n2, n3 = tri.Get()
                triangles.append([offset + n1 - 1,
                                   offset + n2 - 1,
                                   offset + n3 - 1])
            offset += triangulation.NbNodes()

        explorer.Next()

    return np.array(vertices, dtype=np.float64), np.array(triangles, dtype=np.int32)


def main():
    print(f"读取 STEP 文件: {STEP_FILE}")
    vertices, triangles = step_to_mesh(STEP_FILE, deflection=MESH_DEFLECT)
    print(f"  网格顶点数: {len(vertices)}")
    print(f"  三角面片数: {len(triangles)}")

    if len(vertices) == 0:
        raise RuntimeError("网格为空，请检查 STEP 文件或减小 MESH_DEFLECT 值")

    # 单位换算：如果模型是 mm，转成 m（与相机深度单位一致）
    # 先看一下模型的尺寸范围
    bbox_min = vertices.min(axis=0)
    bbox_max = vertices.max(axis=0)
    size = bbox_max - bbox_min
    print(f"  模型尺寸: X={size[0]:.1f}  Y={size[1]:.1f}  Z={size[2]:.1f}  (原始单位)")

    # 自动判断单位：充电口尺寸约 100~200mm，如果最大尺寸 > 10 认为是 mm
    if size.max() > 10:
        print("  检测到单位为 mm，自动转换为 m")
        vertices = vertices / 1000.0
        size = size / 1000.0
    print(f"  转换后尺寸: X={size[0]*1000:.1f}mm  Y={size[1]*1000:.1f}mm  Z={size[2]*1000:.1f}mm")

    # 构建 Open3D 网格
    mesh_o3d = o3d.geometry.TriangleMesh()
    mesh_o3d.vertices  = o3d.utility.Vector3dVector(vertices)
    mesh_o3d.triangles = o3d.utility.Vector3iVector(triangles)
    mesh_o3d.compute_vertex_normals()

    # 从网格表面均匀采样点云（Poisson disk 保证分布均匀）
    print(f"  从网格采样 {N_POINTS} 个点...")
    pcd = mesh_o3d.sample_points_poisson_disk(N_POINTS)

    # 把点云中心移到原点（方便 ICP 对齐）
    center = np.asarray(pcd.points).mean(axis=0)
    pcd.translate(-center)
    print(f"  点云中心已移到原点，中心偏移: {center*1000}")

    # 保存
    o3d.io.write_point_cloud(OUT_PCD, pcd)
    o3d.io.write_point_cloud(OUT_PLY, pcd)
    print(f"已保存:")
    print(f"  {OUT_PCD}  （ICP 模板，{len(pcd.points)} 个点）")
    print(f"  {OUT_PLY}  （可视化用）")

    # 可视化
    print("显示点云（按 Q 关闭）...")
    o3d.visualization.draw_geometries(
        [pcd],
        window_name="充电口模板点云",
        width=800, height=600
    )


if __name__ == "__main__":
    main()
