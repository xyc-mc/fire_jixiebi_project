import numpy as np
import open3d as o3d
import sys
import os
from ultralytics import YOLO
import cv2
from pathlib import Path
from PIL import Image



class PMRCameraCalibration:
    """PMR系列三目相机标定参数类"""
    
    def __init__(self, manual_offset_x=0, manual_offset_y=0):
        # 左相机到RGB相机的旋转矩阵 R_rgb (对应67-75行)
        self.R_rgb = np.array([
            [0.99456604183126, -0.00569193534238, 0.10353160507791],
            [-0.00569193534238, 0.99995193521797, 0.00558185751599],
            [-0.10353160507791, -0.00558185751599, 0.99451810669723]
        ])
        
        # 左相机到RGB相机的平移矩阵 T_rgb (对应76-78行)
        self.T_rgb = np.array([
            -31.14931951415490,
            -0.19268970246425,
            -8.90086102458888
        ])
        
        # RGB相机内参
        self.fx = 1506.72549591614391  # 53行
        self.fy = 1507.87621118303296  # 57行
        self.cx = 962.46320176551478   # 55行
        self.cy = 1227.18940862153447  # 58行
        
        # RGB相机畸变系数 (对应62-66行)
        self.k1 = -0.02485451550629
        self.k2 = -0.01921361572788
        self.p1 = -0.00026809283239
        self.p2 = -0.00118969653118
        self.k3 = 0.00000000000000
        
        # 构建相机内参矩阵
        self.K = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])
        
        # 畸变系数向量
        self.dist_coeffs = np.array([self.k1, self.k2, self.p1, self.p2, self.k3])
        
        # 手动偏移补偿（用于修正配准偏差）
        self.manual_offset_x = manual_offset_x
        self.manual_offset_y = manual_offset_y
    
    def transform_pointcloud_to_rgb(self, points_left):
        """
        将左相机坐标系下的点云转换到RGB相机坐标系
        
        参数:
            points_left: Nx3的numpy数组，左相机坐标系下的点云
        
        返回:
            points_rgb: Nx3的numpy数组，RGB相机坐标系下的点云
        """
        # 应用公式(3): [X_rgb, Y_rgb, Z_rgb]^T = R_rgb * [X, Y, Z]^T + T_rgb
        points_rgb = (self.R_rgb @ points_left.T).T + self.T_rgb
        return points_rgb
    
    def project_3d_to_2d(self, points_3d):
        """
        将3D点投影到RGB图像平面（带畸变校正）
        
        参数:
            points_3d: Nx3的numpy数组，RGB相机坐标系下的3D点
        
        返回:
            pixels: Nx2的numpy数组，对应的像素坐标(u, v)
            valid_mask: 布尔数组，指示哪些点在图像范围内
        """
        # 归一化坐标 (公式4)
        x = points_3d[:, 0] / points_3d[:, 2]
        y = points_3d[:, 1] / points_3d[:, 2]
        
        # 计算径向距离
        r2 = x**2 + y**2
        r4 = r2**2
        r6 = r2**3
        
        # 畸变校正 (公式5)
        x_distorted = x * (1 + self.k1*r2 + self.k2*r4 + self.k3*r6) + \
                      2*self.p1*x*y + self.p2*(r2 + 2*x**2)
        y_distorted = y * (1 + self.k1*r2 + self.k2*r4 + self.k3*r6) + \
                      self.p1*(r2 + 2*y**2) + 2*self.p2*x*y
        
        # 投影到像素坐标 (公式6)
        u = self.fx * x_distorted + self.cx
        v = self.fy * y_distorted + self.cy
        
        # 应用手动偏移补偿
        u = u + self.manual_offset_x
        v = v + self.manual_offset_y
        
        pixels = np.column_stack([u, v])
        
        # 检查是否在图像范围内 (1944x2592)
        valid_mask = (pixels[:, 0] >= 0) & (pixels[:, 0] < 2592) & \
                     (pixels[:, 1] >= 0) & (pixels[:, 1] < 1944) & \
                     (points_3d[:, 2] > 0)  # Z > 0 (在相机前方)
        
        return pixels, valid_mask


class BalancedPointCloudSegmentor:
    
    def __init__(self, yolo_model_path, camera_calib=None):
        self.model = YOLO(yolo_model_path)
        self.calib = camera_calib if camera_calib else PMRCameraCalibration(manual_offset_x=-29, manual_offset_y=53)
    
    def segment_balanced(
        self,
        rgb_path=None,          # 保持向后兼容
        pcd_path=None,          # 保持向后兼容
        rgb_data=None,          # 新增：直接传入图像数据
        pcd_data=None,          # 新增：直接传入点云数据
        output_dir="results_balanced",
        conf_threshold=0.25,
        # 掩膜
        mask_threshold=0.4,
        mask_expand_pixels=5,
        # 深度过滤（温和）
        use_depth_filter=True,
        # 统计过滤
        use_statistical_filter=True,
        # 聚类过滤
        use_cluster_filter=True,
        min_cluster_points=50,     # 最小簇大小
        # 可视化
        visualize=True,
        save_results=True,
        debug_mode=True
    ):
        
        output_dir = Path(output_dir)
        output_dir.mkdir(exist_ok=True, parents=True)
        
        
        
        pcd_original = o3d.io.read_point_cloud(str(pcd_path))
        points_left = np.asarray(pcd_original.points)
        
        image = cv2.imread(str(rgb_path))
        # OpenCV读取的是BGR格式，转换为RGB供YOLO使用
        # if image is not None and image.shape[2] == 3:
        #     image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # 2. YOLO推理
        results = self.model(image, conf=conf_threshold, iou=0.6)
        class_names_dict = self.model.names
        
        if results[0].masks is None:
            return None
        
        segmented_clouds = []
        
        for idx, mask_tensor in enumerate(results[0].masks.data):

            if len(results[0].boxes.cls) > idx:
                class_id = int(results[0].boxes.cls[idx].cpu().numpy())
                class_name = class_names_dict[class_id]  # 这是目标类别名称
                confidence = float(results[0].boxes.conf[idx].cpu().numpy())
            else:
                class_id = -1
                class_name = "unknown"
                confidence = 0.0
            
            # 3. 处理掩膜
            mask_np = mask_tensor.cpu().numpy()
            mask_resized = cv2.resize(mask_np, (image.shape[1], image.shape[0]))
            mask_binary = (mask_resized > mask_threshold).astype(np.uint8)
            
            # 适度扩展掩膜
            if mask_expand_pixels > 0:
                kernel = np.ones((mask_expand_pixels*2+1, mask_expand_pixels*2+1), np.uint8)
                mask_binary = cv2.dilate(mask_binary, kernel, iterations=1)
            
            
            # 保存掩膜可视化
            if debug_mode:
                mask_colored = np.zeros_like(image)
                mask_colored[mask_binary > 0] = [0, 255, 0]  # RGB格式的绿色
                overlay = cv2.addWeighted(image, 0.7, mask_colored, 0.3, 0)
                # 保存时转回BGR格式（OpenCV保存需要BGR）
                overlay_bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
                cv2.imwrite(str(output_dir / f"target_{idx}_mask_overlay.jpg"), overlay_bgr)
            
            # 4. 投影并初步分割
            points_rgb = self.calib.transform_pointcloud_to_rgb(points_left)
            pixels, valid_mask = self.calib.project_3d_to_2d(points_rgb)
            
            segmented_indices = []
            segmented_depths = []
            
            for i in range(len(points_left)):
                if not valid_mask[i]:
                    continue
                u, v = int(pixels[i, 0]), int(pixels[i, 1])
                if 0 <= v < image.shape[0] and 0 <= u < image.shape[1]:
                    if mask_binary[v, u]:
                        segmented_indices.append(i)
                        segmented_depths.append(points_rgb[i, 2])
            
            segmented_indices = np.array(segmented_indices)
            segmented_depths = np.array(segmented_depths)
            
            
            if len(segmented_indices) == 0:
                continue
            
            # 5. 深度过滤（保留较近的点，去除远处背景）
            if use_depth_filter and len(segmented_depths) > 10:
                
                # 使用分位数而不是均值，更鲁棒
                depth_median = np.median(segmented_depths)
                depth_25 = np.percentile(segmented_depths, 25)
                depth_75 = np.percentile(segmented_depths, 75)
                
                
                # 只保留较近的点（小于等于中位数的点）
                # 这样可以去除远处的大圈背景
                depth_max = depth_median + (depth_75 - depth_25) * 0.5  # 中位数 + 0.5倍IQR
                
                
                depth_mask = segmented_depths <= depth_max
                segmented_indices = segmented_indices[depth_mask]
                
            
            if len(segmented_indices) == 0:
                continue
            
            # 6. 创建点云
            segmented_points = points_left[segmented_indices]
            pcd_segmented = o3d.geometry.PointCloud()
            pcd_segmented.points = o3d.utility.Vector3dVector(segmented_points)
            
            # 7. 统计离群点过滤
            if use_statistical_filter and len(segmented_points) > 50:
                pcd_segmented, ind = pcd_segmented.remove_statistical_outlier(
                    nb_neighbors=20, std_ratio=2.0
                )
                segmented_points = np.asarray(pcd_segmented.points)
            
            # 8. 聚类过滤
            if use_cluster_filter and len(segmented_points) > min_cluster_points:
                labels = np.array(pcd_segmented.cluster_dbscan(
                    eps=8.0, min_points=10, print_progress=False
                ))
                
                if len(labels) > 0 and labels.max() >= 0:
                    unique_labels, counts = np.unique(labels[labels >= 0], return_counts=True)
                    
                    # 过滤掉太小的簇
                    valid_clusters = unique_labels[counts >= min_cluster_points]
                    
                    if len(valid_clusters) > 0:
                        
                        # 保留最大的簇
                        largest_cluster_idx = np.argmax(counts[np.isin(unique_labels, valid_clusters)])
                        largest_cluster = valid_clusters[largest_cluster_idx]
                        
                        cluster_mask = labels == largest_cluster
                        pcd_segmented = pcd_segmented.select_by_index(
                            np.where(cluster_mask)[0]
                        )
                        segmented_points = np.asarray(pcd_segmented.points)

            
            # 9. 最终结果
            if len(segmented_points) > 0:
                pcd_segmented.paint_uniform_color([0.9, 0.9, 0.9])
                
                center = segmented_points.mean(axis=0)
                bbox_min = segmented_points.min(axis=0)
                bbox_max = segmented_points.max(axis=0)
                bbox_size = bbox_max - bbox_min
                
                
                segmented_clouds.append({
                    'points': segmented_points,
                    'pcd': pcd_segmented,
                    'center': center,
                    'bbox_size': bbox_size,
                    'target_id': idx,
                    'class_name': class_name,
                    'confidence': confidence
                })
        
        # 保存
        if save_results and len(segmented_clouds) > 0:
            
            annotated = results[0].plot()
            # YOLO的plot()返回的是RGB格式，保存时转为BGR
            annotated_bgr = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(output_dir / "yolo_detection.jpg"), annotated_bgr)
            
            for seg_cloud in segmented_clouds:
                idx = seg_cloud['target_id']
                pcd_path = output_dir / f"target_{idx}_balanced.pcd"
                o3d.io.write_point_cloud(str(pcd_path), seg_cloud['pcd'])
                
                # 保存信息
                info_path = output_dir / f"target_{idx}_info.txt"
                with open(info_path, 'w', encoding='utf-8') as f:
                    f.write(f"分割结果\n")
                    f.write("=" * 40 + "\n")
                    f.write(f"类别: {class_name}\n")
                    f.write(f"点数: {len(seg_cloud['points'])}\n")
                    f.write(f"中心 (mm): {seg_cloud['center']}\n")
                    f.write(f"尺寸 (mm): {seg_cloud['bbox_size']}\n")
        
        # 可视化
        if visualize and len(segmented_clouds) > 0:
            vis_geometries = []
            
            pcd_vis_original = o3d.geometry.PointCloud(pcd_original)
            pcd_vis_original.paint_uniform_color([0.7, 0.7, 0.7])
            vis_geometries.append(pcd_vis_original)
            
            for seg_cloud in segmented_clouds:
                pcd_vis = o3d.geometry.PointCloud(seg_cloud['pcd'])
                pcd_vis.translate([0, 0, 50])
                vis_geometries.append(pcd_vis)
            
            o3d.visualization.draw_geometries(
                vis_geometries,
                window_name="平衡版分割结果",
                width=1280,
                height=720
            )
        
        
        return segmented_clouds


def main():
    """主函数"""
    
    model_path = r"/home/ubuntu/fire_jixiebi_ws/src/point_rgb_address/config/best.pt"
    rgb_image = r"/home/ubuntu/fire_jixiebi_ws/src/point_rgb_address/camera_data/capture_rgb.jpg"
    pointcloud = r"/home/ubuntu/fire_jixiebi_ws/src/point_rgb_address/camera_data/capture.pcd"
    
    
    segmentor = BalancedPointCloudSegmentor(model_path)
    
    results = segmentor.segment_balanced(
        rgb_path=rgb_image,
        pcd_path=pointcloud,
        output_dir=r"/home/ubuntu/fire_jixiebi_ws/src/point_rgb_address/results",
        conf_threshold=0.25,
        
        # 掩膜（放宽以保留完整轮廓）
        mask_threshold=0.25,        # 进一步降低阈值
        mask_expand_pixels=10,      # 增加扩展到10像素
        
        # 深度过滤（严格，去除远处背景）
        use_depth_filter=True,
        
        # 统计和聚类过滤（减少过滤强度）
        use_statistical_filter=False,  # 关闭统计过滤，避免边缘点被误删
        use_cluster_filter=True,
        min_cluster_points=30,      # 降低最小簇大小
        
        visualize=True,
        save_results=True,
        debug_mode=True
    )
    
    if results:
        print(f"\n✅ 成功分割 {len(results)} 个目标")
        for seg in results:
            print(f"\n目标 {seg['target_id']}:")
            print(f"  点数: {len(seg['points'])}")
            print(f"  中心: ({seg['center'][0]:.1f}, {seg['center'][1]:.1f}, {seg['center'][2]:.1f})")
            print(f"  尺寸: {seg['bbox_size'][0]:.1f} x {seg['bbox_size'][1]:.1f} x {seg['bbox_size'][2]:.1f} mm")


if __name__ == "__main__":
    main()
