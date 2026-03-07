import rclpy
from rclpy.node import Node
from base_msgs.msg import ControlMsg
from .segment_balanced import *
import os
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import String
import numpy as np
from .centralized_computing import *
from .calculate_rotation import FeatureMatchAligner
from .segment_dilate import *

class AddressControl(Node):
    def __init__(self):
        super().__init__('address_control_node')

        self.declare_parameter('workspace_dir', '/home/ubuntu/fire_jixiebi_ws')
        self.workspace_dir = self.get_parameter('workspace_dir').value
        self.config_dir = os.path.join(self.workspace_dir, 'src/point_rgb_address/config')
        self.model_path = os.path.join(self.workspace_dir, 'src/point_rgb_address/config/best.pt')
        self.results_dir = os.path.join(self.workspace_dir, 'src/point_rgb_address/results')
        self.camera_data_dir = os.path.join(self.workspace_dir, 'src/point_rgb_address/camera_data')
        self.rgb_path = os.path.join(self.workspace_dir, 'src/point_rgb_address/camera_data/capture_rgb.jpg')
        self.pcd_path = os.path.join(self.workspace_dir, 'src/point_rgb_address/camera_data/capture.pcd')
        self.newpcd_path = os.path.join(self.results_dir, 'target_0_balanced.pcd')
        self.match_result_path = os.path.join(self.results_dir, 'match_result.jpg')
        self.template_img_path = os.path.join(self.workspace_dir, 'src/point_rgb_address/config/master_aligned.jpg')

        self.control_pub = self.create_publisher(ControlMsg, '/sport_control', 10)
        self.control_sub = self.create_subscription(ControlMsg, '/address_control', self.address_callback, 10)

        self.aligner = FeatureMatchAligner(self, self.template_img_path)
        
        self.process_pub = self.create_publisher(String, '/process_data', 10)
        self.error_pub = self.create_publisher(String, '/error_data', 10)

        self.segment_rgb_data_path = os.path.join(self.workspace_dir, 'src/point_rgb_address/camera_data/segment_rgb.jpg')


    def address_callback(self, data):
        try:
            process = String()
            if data.frequency == 1:   
                process.data = 'address_first'
                self.process_pub.publish(process)
            elif data.frequency == 2:
                process.data = 'address_second'
                self.process_pub.publish(process)
            control_msg = ControlMsg()
            if data.switch_control:
                    
                self.get_logger().info(f"第{data.frequency}次处理")
                segmentor = BalancedPointCloudSegmentor(self.model_path)
                    
                results = segmentor.segment_balanced(
                    rgb_data=None,      # 传入图像数据
                    pcd_data=None,     # 传入点云数据
                    rgb_path=self.rgb_path,
                    pcd_path=self.pcd_path,
                    output_dir=self.results_dir,
                    conf_threshold=0.7,
                        
                    # 掩膜（放宽以保留完整轮廓）
                    mask_threshold=0.25,
                    mask_expand_pixels=10,
                        
                    # 深度过滤（严格，去除远处背景）
                    use_depth_filter=True,
                        
                    # 统计和聚类过滤（减少过滤强度）
                    use_statistical_filter=False,
                    use_cluster_filter=True,
                    min_cluster_points=30,
                        
                    visualize=False,
                    save_results=True,
                    debug_mode=True
                )
                    
                if results:
                    self.get_logger().info(f"✅ 成功分割 {len(results)} 个目标")
                    if len(results) != 1:
                        self.get_logger().error("目标分割数量异常，取消本次任务")
                        return
                    for seg in results:
                        if data.frequency == 1:
                            control_msg.switch_control = True
                            control_msg.class_name = seg['class_name']
                            control_msg.center = [seg['center'][0], seg['center'][1], seg['center'][2]]
                            control_msg.frequency = data.frequency
                            self.get_logger().info(f"  类别: {seg['class_name']}:")
                            self.get_logger().info(f"  点数: {len(seg['points'])}")
                            self.get_logger().info(f"  中心: ({seg['center'][0]:.1f}, {seg['center'][1]:.1f}, {seg['center'][2]:.1f})")
                        elif data.frequency >= 2:
                            control_msg.switch_control = True
                            control_msg.class_name = seg['class_name']
                            center3d, normal = process_detector(self.newpcd_path, visualize=True)
                            control_msg.center = [center3d[0], center3d[1], center3d[2]]
                            control_msg.angle = [normal[0], normal[1], normal[2]]
                            control_msg.frequency = data.frequency
                            if seg['class_name'] == 'base':
                                crop_and_mask_object(self.rgb_path, self.segment_rgb_data_path, self.model_path, dilation_size=10)
                                rotation_deg = self.aligner.calculate_rotation(self.segment_rgb_data_path, self.match_result_path)
                                if rotation_deg <= 0:
                                    control_msg.rotate_direction = '逆时针'
                                    control_msg.rotate_angle = -rotation_deg
                                elif rotation_deg > 0:
                                    control_msg.rotate_direction = '顺时针'
                                    control_msg.rotate_angle = rotation_deg
                                else:
                                    self.get_logger().error(f'角度计算失败')
                            self.get_logger().info(f"  类别: {seg['class_name']}:")
                            self.get_logger().info(f"  点数: {len(seg['points'])}")
                            self.get_logger().info(f"  中心: ({center3d[0]}, {center3d[1]}, {center3d[2]})")
                            self.get_logger().info(f"  法向量: ({normal[0]}, {normal[1]}, {normal[2]})")

                else:
                    self.get_logger().warning("分割失败，未检测到目标")
                
            self.control_pub.publish(control_msg)
        except Exception as e:
            self.error_pub.publish(process)
            self.get_logger().error('数据处理出现未知错误，取消本次任务')
            return


    
def main():
    rclpy.init()
    address_point_rgb = AddressControl()
    rclpy.spin(address_point_rgb)
    rclpy.shutdown()

if __name__ == '__main__':
    main()