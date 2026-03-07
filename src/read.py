import open3d as o3d
import numpy as np
import os
import sys

def read_pcd_file(file_path):
    """
    读取PCD文件并返回点云数据
    
    参数:
        file_path: PCD文件的路径
    
    返回:
        点云对象和点云数据的NumPy数组
    """
    # 检查文件是否存在
    if not os.path.exists(file_path):
        print(f"错误: 文件 {file_path} 不存在!")
        sys.exit(1)
    
    try:
        # 使用Open3D读取PCD文件
        pcd = o3d.io.read_point_cloud(file_path)
        
        # 将点云数据转换为NumPy数组
        points = np.asarray(pcd.points)
        
        print(f"成功读取点云数据，共有 {len(points)} 个点")
        print(f"点云数据的形状: {points.shape}")
        
        # 显示前5个点的坐标
        if len(points) > 0:
            print("\n前5个点的坐标:")
            for i in range(min(5, len(points))):
                print(f"点 {i+1}: {points[i]}")
        
        return pcd, points
    
    except Exception as e:
        print(f"读取PCD文件时出错: {e}")
        sys.exit(1)

def visualize_point_cloud(pcd):
    """
    可视化点云数据
    
    参数:
        pcd: Open3D点云对象
    """
    # 可视化点云
    print("\n正在可视化点云数据，关闭窗口继续...")
    o3d.visualization.draw_geometries([pcd])





if __name__ == "__main__":
    # 设置PCD文件路径11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
    pcd_file = os.path.join("/home/neepu/fire_jixiebi_ws/src/point_rgb_address/camera_data/capture.pcd")
    # 设置PCD文件路径1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111









    # 读取PCD文件
    pcd, points = read_pcd_file(pcd_file)
    
    # 可视化点云数据
    visualize_point_cloud(pcd)