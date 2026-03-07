import numpy as np
import open3d as o3d
import os

def read_pcd_binary(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"文件未找到: {file_path}")
        
    with open(file_path, 'rb') as f:
        header = []
        while True:
            line = f.readline().decode('ascii').strip()
            header.append(line)
            if line.startswith('DATA binary'):
                break
        
        points_num = 0
        for line in header:
            if line.startswith('POINTS'):
                points_num = int(line.split()[1])
        
        data = f.read()
        dt = np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('rgb', 'f4')])
        pc_data = np.frombuffer(data, dtype=dt)[:points_num]
        return np.stack([pc_data['x'], pc_data['y'], pc_data['z']], axis=1)

def fit_circle_2d(p_2d):
    """基于最小二乘法的二维圆拟合"""
    x, y = p_2d[:, 0], p_2d[:, 1]
    A_mat = np.stack([x, y, np.ones_like(x)], axis=1)
    B_vec = x**2 + y**2
    # 求解 Ax = B
    res, _, _, _ = np.linalg.lstsq(A_mat, B_vec, rcond=None)
    xc, yc = res[0] / 2, res[1] / 2
    r = np.sqrt(res[2] + xc**2 + yc**2)
    return xc, yc, r

def create_3d_circle_lines(center_3d, u, v, radius, segments=100):
    """在 3D 空间中根据圆心、基底向量和半径生成红色圆周线集"""
    theta = np.linspace(0, 2 * np.pi, segments)
    circle_pts = [center_3d + radius * np.cos(t) * u + radius * np.sin(t) * v for t in theta]
    
    lines = [[i, (i + 1) % segments] for i in range(segments)]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(circle_pts)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.paint_uniform_color([1, 0, 0])  # 红色线
    return line_set

def process_detector(file_path, visualize=True):
    """
    处理点云主函数
    """
    # 1. 加载数据
    points = read_pcd_binary(file_path)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 2. RANSAC 拟合平面 (提取主表面)
    plane_model, inliers = pcd.segment_plane(distance_threshold=1.5,
                                             ransac_n=3,
                                             num_iterations=1000)
    a, b, c, d = plane_model
    normal = np.array([a, b, c]) # 平面法向量

    # 3. 建立局部坐标系投影至 2D
    origin_on_plane = -normal * d
    ref = np.array([1, 0, 0]) if abs(a) < 0.8 else np.array([0, 1, 0])
    u = np.cross(normal, ref)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)

    # 获取平面内点并投影
    inlier_pts_3d = points[inliers]
    p_2d = np.stack([np.dot(inlier_pts_3d - origin_on_plane, u), 
                     np.dot(inlier_pts_3d - origin_on_plane, v)], axis=1)

    # 4. 拟合中心坐标
    xc_2d, yc_2d, radius = fit_circle_2d(p_2d)
    center_3d = origin_on_plane + xc_2d * u + yc_2d * v

    # 5. 打印计算结果
    print("\n" + "="*40)
    print("几何中心计算完成")
    print("-" * 40)
    print(f"拟合平面法向量 (Normal):  [{a:.6f}, {b:.6f}, {c:.6f}]")
    print(f"平面方程参数 (d):          {d:.6f}")
    print(f"图形中心坐标 (X, Y, Z):   {center_3d}")
    print(f"参与拟合的点数 (Inliers):  {len(inliers)}")
    print("="*40 + "\n")

    # 6. 可视化模块
    if visualize:
        # 配置点云显示
        pcd.paint_uniform_color([0.5, 0.5, 0.5]) # 背景点灰色
        inlier_pcd = pcd.select_by_index(inliers)
        inlier_pcd.paint_uniform_color([0, 0.5, 1]) # 平面内点蓝色

        # 配置中心标记和圆周
        center_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.8)
        center_marker.paint_uniform_color([1, 0, 0]) # 中心球红色
        center_marker.translate(center_3d)
        
        circle_lines = create_3d_circle_lines(center_3d, u, v, radius)

        print("正在启动 Open3D 可视化窗口... (按 Q 键退出)")
        o3d.visualization.draw_geometries([pcd, inlier_pcd, circle_lines, center_marker],
                                          window_name=f"中心检测: {os.path.basename(file_path)}",
                                          width=1000, height=700)
    
    return center_3d, normal

if __name__ == "__main__":
    PCD_FILE = '/home/neepu/fire_jixiebi_ws/src/point_rgb_address/results/target_0_balanced.pcd'
    
    process_detector(PCD_FILE, visualize=True)