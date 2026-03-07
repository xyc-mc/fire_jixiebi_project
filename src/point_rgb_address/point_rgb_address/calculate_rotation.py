import cv2
import numpy as np
import os

class FeatureMatchAligner:
    def __init__(self, node, template_img_path="master_aligned.jpg"):
        self.sift = cv2.SIFT_create()
        self.template_kp = None
        self.template_des = None
        self.template_img = None
        self.template_path = template_img_path
        self.node = node
        
        if os.path.exists(template_img_path):
            self.template_img = cv2.imread(template_img_path)
            gray = cv2.cvtColor(self.template_img, cv2.COLOR_BGR2GRAY)
            self.template_kp, self.template_des = self.sift.detectAndCompute(gray, None)
            self.node.get_logger().info(f"模板加载成功：提取到 {len(self.template_kp)} 个特征点")
        else:
            self.node.get_logger().info("找不到模板文件")

    def preprocess_image(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        enhanced_gray = clahe.apply(gray)
        h, w = enhanced_gray.shape
        if w < 500:
            scale_factor = 2.0
            enhanced_gray = cv2.resize(enhanced_gray, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_CUBIC)
        return enhanced_gray

    def calculate_rotation(self, current_img_path, output_res_path="match_result.jpg"):
        if self.template_des is None:
            self.node.get_logger().info("模板有问题")
            return None

        curr_img = cv2.imread(current_img_path)
        if curr_img is None: 
            self.node.get_logger().info(f"错误：无法读取图片 {current_img_path}")
            return None
        
        gray_curr = self.preprocess_image(curr_img)
        cv2.imwrite('/home/neepu/fire_jixiebi_ws/src/point_rgb_address/results/debug.jpg', gray_curr)
        kp_curr, des_curr = self.sift.detectAndCompute(gray_curr, None)
        
        if des_curr is None: 
            self.node.get_logger().info("未能提取特征点")
            return None

        # 1. 特征点匹配
        flann = cv2.FlannBasedMatcher(dict(algorithm=1, trees=5), dict(checks=50))
        matches = flann.knnMatch(self.template_des, des_curr, k=2)

        # 2. 筛选高质量匹配点
        good_matches = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)

        if len(good_matches) < 4:
            self.node.get_logger().info(f"匹配点不足 ({len(good_matches)}), 无法计算角度")
            return None

        # 3. 利用 RANSAC 剔除误匹配
        src_pts = np.float32([self.template_kp[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp_curr[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 10.0)

        if M is not None:
            # 4. 从变换矩阵中提取旋转角度
            # M[0,0] 是 cos(theta), M[1,0] 是 sin(theta)
            rotation_rad = np.arctan2(M[1, 0], M[0, 0])
            rotation_deg = np.degrees(rotation_rad)
            
            # 5. 可视化并保存图片
            matches_mask = mask.ravel().tolist()
            res_img = cv2.drawMatches(self.template_img, self.template_kp, 
                                     curr_img, kp_curr, 
                                     good_matches, None, 
                                     matchesMask=matches_mask, 
                                     flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            
            # 在结果图上写字
            txt = f"Detected Rotation: {rotation_deg:.2f} Deg"
            cv2.putText(res_img, txt, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            
            # 保存结果图
            cv2.imwrite(output_res_path, res_img)
            self.node.get_logger().info(f"计算完成！角度: {rotation_deg:.2f} 度")
            self.node.get_logger().info(f"结果图已保存至: {output_res_path}")
            
            return rotation_deg
        
        return None

if __name__ == "__main__":
    aligner = FeatureMatchAligner()
    # 特征点匹配，旋转角度计算
    angle = aligner.calculate_rotation("/home/ubuntu/fire_jixiebi_ws/output_cropped_object.jpg")