#!/usr/bin/env python3
"""
平面法向量转机械臂RPY角节点
功能：将点云相机检测的平面法向量转换为机械臂末端的RPY角
"""
import rclpy
from rclpy.node import Node
import numpy as np
import math
import os
from ament_index_python.packages import get_package_share_directory
from base_msgs.msg import ControlMsg
from geometry_msgs.msg import PoseStamped, Quaternion
from elite_msgs.msg import RobotState
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class PlaneNormalToRPYNode(Node):
    def __init__(self):
        super().__init__('plane_normal_to_rpy_node')
        
        # 加载手眼标定矩阵
        self.EHArray = self.load_handeye_calibration()
        self.R_end_camera = None
        self.extract_rotation_matrix()
        
        # 当前TCP位姿（欧拉角格式：[x, y, z, rx, ry, rz]）
        self.current_tcp_pose = None
        self.pose_received = False
        
        # 订阅相机检测的平面法向量
        self.camera_sub = self.create_subscription(
            ControlMsg,
            '/sport_control',  # 假设消息中包含平面法向量
            self.plane_normal_callback,
            10
        )
        
        # 订阅机械臂状态
        self.robot_state_sub = self.create_subscription(
            RobotState,
            '/robot_state',
            self.robot_state_callback,
            10
        )
        
        # 发布目标位姿（包含计算出的RPY角）
        self.target_pub = self.create_publisher(
            PoseStamped,
            '/target_pose_with_rpy',
            10
        )
        
        # 发布纯姿态消息（仅旋转）
        self.orientation_pub = self.create_publisher(
            Quaternion,
            '/target_orientation',
            10
        )
        
        # 初始化日志
        self.get_logger().info("平面法向量转RPY角节点已启动")
        
        # 参考方向设置
        self.reference_direction = np.array([1.0, 0.0, 0.0])  # 世界坐标系X轴
        
        # 输出转换矩阵信息
        self.log_transform_info()
    
    def load_handeye_calibration(self):
        """从文件加载手眼标定参数"""
        try:
            package_path = get_package_share_directory('handeye_coord_transformer')
            config_path = os.path.join(package_path, 'config', 'EHMatrix.txt')
            
            EHArray = np.zeros(16, dtype=np.float32)
            
            with open(config_path, 'r') as file:
                data = file.read().strip().split()
                
                if len(data) < 16:
                    self.get_logger().error(f"文件数据不足16个: {len(data)}")
                    raise ValueError("文件数据不足16个")
                
                for i in range(16):
                    EHArray[i] = float(data[i])
            
            self.get_logger().info("手眼矩阵加载成功")
            return EHArray
            
        except Exception as e:
            self.get_logger().error(f"加载标定文件失败: {e}")
            return np.eye(4, dtype=np.float32).flatten()
    
    def extract_rotation_matrix(self):
        """从手眼矩阵中提取旋转矩阵"""
        EH_matrix = np.array(self.EHArray).reshape(4, 4)
        self.R_end_camera = EH_matrix[:3, :3]
        
        # 验证旋转矩阵的有效性
        det = np.linalg.det(self.R_end_camera)
        if abs(det - 1.0) > 0.001:
            self.get_logger().warn(f"旋转矩阵行列式不为1: {det:.6f}")
        
        self.get_logger().info(f"旋转矩阵提取完成")
    
    def log_transform_info(self):
        """记录转换信息"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("平面法向量转RPY角配置:")
        self.get_logger().info(f"参考方向: {self.reference_direction}")
        self.get_logger().info(f"手眼旋转矩阵形状: {self.R_end_camera.shape}")
        self.get_logger().info("=" * 60)
    
    def robot_state_callback(self, msg):
        """处理机器人状态的回调函数"""
        try:
            if len(msg.machine_pose) >= 6:
                x_mm = msg.machine_pose[0]
                y_mm = msg.machine_pose[1]
                z_mm = msg.machine_pose[2]
                rx_rad = msg.machine_pose[3]
                ry_rad = msg.machine_pose[4]
                rz_rad = msg.machine_pose[5]
                
                self.current_tcp_pose = [x_mm, y_mm, z_mm, rx_rad, ry_rad, rz_rad]
                self.pose_received = True
                
                self.get_logger().debug(f"当前TCP位姿更新")
                
        except Exception as e:
            self.get_logger().error(f"处理机器人状态失败: {e}")
    
    def normalize_vector(self, vector):
        """归一化向量"""
        norm = np.linalg.norm(vector)
        if norm < 1e-6:
            return vector
        return vector / norm
    
    def camera_normal_to_rpy(self, normal_camera):
        """
        核心函数：将相机坐标系下的法向量转换为末端RPY角
        
        参数：
        normal_camera: 相机坐标系下的平面法向量（单位向量）
        
        返回：
        rpy: [roll, pitch, yaw] 弧度
        R_new: 新的末端旋转矩阵
        """
        # 步骤1：法向量从相机坐标系转换到末端坐标系
        n_end = self.R_end_camera @ normal_camera
        n_end = self.normalize_vector(n_end)
        
        self.get_logger().info(f"末端坐标系法向量: [{n_end[0]:.4f}, {n_end[1]:.4f}, {n_end[2]:.4f}]")
        
        # 步骤2：构建新的旋转矩阵
        # Z轴 = 法向量
        z_axis = -n_end
        
        # 检查法向量与参考方向的夹角
        ref_dir = self.reference_direction.copy()
        dot_product = np.abs(np.dot(z_axis, ref_dir))
        
        # 如果法向量与参考方向太接近，换一个参考方向
        if dot_product > 0.99:  # 接近平行
            ref_dir = np.array([0.0, 1.0, 0.0])  # 换用Y轴
            self.get_logger().debug("使用备用参考方向Y轴")
        
        # X轴 = 参考方向在平面上的投影
        x_axis_proj = ref_dir - np.dot(ref_dir, z_axis) * z_axis
        x_axis_proj_norm = np.linalg.norm(x_axis_proj)
        
        if x_axis_proj_norm < 1e-6:
            # 投影太小，选择任意正交向量
            if abs(z_axis[0]) < abs(z_axis[1]) and abs(z_axis[0]) < abs(z_axis[2]):
                x_axis = np.array([1.0, 0.0, 0.0])
            elif abs(z_axis[1]) < abs(z_axis[2]):
                x_axis = np.array([0.0, 1.0, 0.0])
            else:
                x_axis = np.array([0.0, 0.0, 1.0])
            
            # 正交化
            x_axis = x_axis - np.dot(x_axis, z_axis) * z_axis
        else:
            x_axis = x_axis_proj / x_axis_proj_norm
        
        x_axis = self.normalize_vector(x_axis)
        
        # Y轴 = Z × X（右手系）
        y_axis = np.cross(z_axis, x_axis)
        y_axis = self.normalize_vector(y_axis)
        
        # 构建旋转矩阵 [X, Y, Z]
        R_new = np.column_stack([x_axis, y_axis, z_axis])
        
        # 验证旋转矩阵
        self.validate_rotation_matrix(R_new, n_end)
        
        # 步骤3：转换为RPY角（ZYX欧拉角）
        # 注意：这里的RPY是绕固定坐标系的旋转顺序
        rpy = self.rotation_matrix_to_rpy_zyx(R_new)
        
        return rpy, R_new
    
    def validate_rotation_matrix(self, R, expected_z):
        """验证旋转矩阵的有效性"""
        # 检查正交性
        RRT = R @ R.T
        identity = np.eye(3)
        ortho_error = np.max(np.abs(RRT - identity))
        
        # 检查行列式
        det = np.linalg.det(R)
        
        # 检查Z轴方向
        z_from_R = R[:, 2]
        z_error = np.arccos(np.clip(np.dot(z_from_R, expected_z), -1.0, 1.0))
        
        self.get_logger().debug(f"旋转矩阵验证:")
        self.get_logger().debug(f"  正交性误差: {ortho_error:.6f}")
        self.get_logger().debug(f"  行列式: {det:.6f}")
        self.get_logger().debug(f"  Z轴方向误差: {np.degrees(z_error):.2f}°")
        
        if ortho_error > 0.001 or abs(det - 1.0) > 0.001:
            self.get_logger().warn("旋转矩阵可能存在问题")
    
    def rotation_matrix_to_rpy_zyx(self, R):
        """
        旋转矩阵转ZYX欧拉角（RPY）
        旋转顺序：先绕Z轴转(yaw)，再绕Y轴转(pitch)，最后绕X轴转(roll)
        
        公式：
        R = Rz(yaw) * Ry(pitch) * Rx(roll)
        """
        # 提取矩阵元素
        r11, r12, r13 = R[0, 0], R[0, 1], R[0, 2]
        r21, r22, r23 = R[1, 0], R[1, 1], R[1, 2]
        r31, r32, r33 = R[2, 0], R[2, 1], R[2, 2]
        
        # 计算pitch
        pitch = math.atan2(-r31, math.sqrt(r11**2 + r21**2))
        
        # 检查万向节锁（pitch接近±90°）
        if abs(abs(pitch) - math.pi/2) < 1e-6:
            # 奇异情况
            roll = 0.0
            if pitch > 0:  # pitch = 90°
                yaw = math.atan2(r12, r22)
            else:  # pitch = -90°
                yaw = -math.atan2(r12, r22)
        else:
            # 非奇异情况
            roll = math.atan2(r32 / math.cos(pitch), r33 / math.cos(pitch))
            yaw = math.atan2(r21 / math.cos(pitch), r11 / math.cos(pitch))
        
        return [roll, pitch, yaw]  # [rx, ry, rz]
    
    def calculate_target_position(self, plane_point_camera=None, offset_distance=10.0):
        """
        计算目标位置
        
        参数：
        plane_point_camera: 相机坐标系下的平面上一点（可选）
        offset_distance: 沿法向量方向的偏移距离（毫米）
        
        返回：
        target_position: 目标位置 [x, y, z]（毫米）
        """
        if plane_point_camera is None:
            # 如果没有平面点，使用当前TCP位置
            if self.current_tcp_pose:
                return np.array(self.current_tcp_pose[:3])
            else:
                return np.array([0.0, 0.0, 0.0])
        
        # TODO: 如果提供平面点，可以计算更精确的目标位置
        # 这里简化处理，使用当前TCP位置
        return np.array(self.current_tcp_pose[:3])
    
    def plane_normal_callback(self, msg):
        """处理平面法向量的回调函数"""
        if not self.pose_received or self.current_tcp_pose is None:
            self.get_logger().warn("尚未收到TCP位姿，无法进行法向量转换")
            return
        
        try:
            # 假设ControlMsg消息包含法向量
            # 这里需要根据实际的消息结构调整
            if len(msg.angle) >= 3:
                # 假设法向量存储在center字段的前三个元素
                # 或者消息中有专门的normal字段
                normal_camera = np.array([msg.angle[0], msg.angle[1], msg.angle[2]])
            else:
                self.get_logger().error("消息中未找到法向量数据")
                return
            
            # 归一化输入法向量
            normal_camera = self.normalize_vector(normal_camera)
            
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"收到相机法向量: [{normal_camera[0]:.4f}, "
                                 f"{normal_camera[1]:.4f}, {normal_camera[2]:.4f}]")
            
            # 转换为RPY角
            rpy, R_new = self.camera_normal_to_rpy(normal_camera)
            
            # 提取RPY角
            roll_rad, pitch_rad, yaw_rad = rpy
            
            # 计算目标位置
            target_position = self.calculate_target_position()
            
            # 创建并发布目标位姿消息
            target_msg = PoseStamped()
            target_msg.header.stamp = self.get_clock().now().to_msg()
            target_msg.header.frame_id = "base_link"
            
            # 设置位置（使用当前TCP位置或计算出的位置）
            target_msg.pose.position.x = float(target_position[0])
            target_msg.pose.position.y = float(target_position[1])
            target_msg.pose.position.z = float(target_position[2])
            
            # 将RPY角转换为四元数
            quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
            target_msg.pose.orientation.x = float(quaternion[0])
            target_msg.pose.orientation.y = float(quaternion[1])
            target_msg.pose.orientation.z = float(quaternion[2])
            target_msg.pose.orientation.w = float(quaternion[3])
            
            # 发布消息
            self.target_pub.publish(target_msg)
            
            # 同时发布纯姿态消息
            orientation_msg = Quaternion()
            orientation_msg.x = float(quaternion[0])
            orientation_msg.y = float(quaternion[1])
            orientation_msg.z = float(quaternion[2])
            orientation_msg.w = float(quaternion[3])
            self.orientation_pub.publish(orientation_msg)
            
            # 输出转换结果
            self.log_conversion_results(normal_camera, rpy, R_new, target_position)
            
        except Exception as e:
            self.get_logger().error(f"法向量转换失败: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def log_conversion_results(self, normal_camera, rpy, R_new, target_position):
        """记录转换结果"""
        roll_rad, pitch_rad, yaw_rad = rpy
        
        # 转换为度
        roll_deg = math.degrees(roll_rad)
        pitch_deg = math.degrees(pitch_rad)
        yaw_deg = math.degrees(yaw_rad)
        
        # 提取旋转矩阵的Z轴
        z_from_R = R_new[:, 2]
        
        # 计算转换误差
        n_end_expected = self.R_end_camera @ normal_camera
        n_end_expected = self.normalize_vector(n_end_expected)
        
        angle_error = math.degrees(np.arccos(np.clip(np.dot(z_from_R, n_end_expected), -1.0, 1.0)))
        
        self.get_logger().info("法向量转换结果:")
        self.get_logger().info(f"  输入法向量: [{normal_camera[0]:.4f}, {normal_camera[1]:.4f}, {normal_camera[2]:.4f}]")
        self.get_logger().info(f"  计算出的RPY角:")
        self.get_logger().info(f"    Roll (X轴): {roll_rad:.4f} rad = {roll_deg:.2f}°")
        self.get_logger().info(f"    Pitch (Y轴): {pitch_rad:.4f} rad = {pitch_deg:.2f}°")
        self.get_logger().info(f"    Yaw (Z轴): {yaw_rad:.4f} rad = {yaw_deg:.2f}°")
        self.get_logger().info(f"  目标位置: [{target_position[0]:.1f}, {target_position[1]:.1f}, {target_position[2]:.1f}] mm")
        self.get_logger().info(f"  法向量对齐误差: {angle_error:.2f}°")
        self.get_logger().info("=" * 60)
        
        # 输出当前TCP姿态对比
        if self.current_tcp_pose:
            current_rx_deg = math.degrees(self.current_tcp_pose[3])
            current_ry_deg = math.degrees(self.current_tcp_pose[4])
            current_rz_deg = math.degrees(self.current_tcp_pose[5])
            
            self.get_logger().info(f"当前TCP姿态: [{current_rx_deg:.1f}°, {current_ry_deg:.1f}°, {current_rz_deg:.1f}°]")
            self.get_logger().info(f"姿态变化: ΔRoll={roll_deg-current_rx_deg:.1f}°, "
                                 f"ΔPitch={pitch_deg-current_ry_deg:.1f}°, "
                                 f"ΔYaw={yaw_deg-current_rz_deg:.1f}°")
    


def main(args=None):
    rclpy.init(args=args)
    node = PlaneNormalToRPYNode()
    
    # 可选：运行测试
    # node.test_conversion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被用户中断")
    except Exception as e:
        node.get_logger().error(f"节点运行出错: {e}")
        import traceback
        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()