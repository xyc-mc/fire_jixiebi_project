#!/usr/bin/env python3
"""
手眼标定坐标转换节点（修正版）
功能：正确处理手眼标定矩阵和机器人位姿的坐标转换
"""
import rclpy
from rclpy.node import Node
import numpy as np
import yaml
import os
import math
from ament_index_python.packages import get_package_share_directory
from base_msgs.msg import ControlMsg
from geometry_msgs.msg import PoseStamped
from elite_msgs.msg import RobotState
from tf_transformations import quaternion_from_euler


class HandEyeTransformNode(Node):
    def __init__(self):
        super().__init__('handeye_transform_node')
        
        # 加载标定参数（从EHMatrix.txt读取）
        self.EHArray = self.load_handeye_calibration()
        
        # 当前TCP位姿（欧拉角格式：[x, y, z, rx, ry, rz]）
        # 注意：这是TCP在基座坐标系下的位姿
        self.current_tcp_pose = None  # 单位：毫米和弧度
        self.pose_received = False
        
        # 订阅相机坐标
        self.camera_sub = self.create_subscription(
            ControlMsg,
            '/sport_control',
            self.camera_point_callback,
            10
        )
        
        # 订阅机械臂状态（包含TCP位姿）
        self.robot_state_sub = self.create_subscription(
            RobotState,
            '/robot_state',
            self.robot_state_callback,
            10
        )
        
        # 发布目标位姿
        self.target_pub = self.create_publisher(
            PoseStamped,
            '/target_pose',
            10
        )
        
        # 初始化日志
        self.get_logger().info("手眼标定转换节点已启动（修正版）")
        self.analyze_eh_matrix()
    
    def load_handeye_calibration(self):
        """从文件加载手眼标定参数（4x4变换矩阵）"""
        try:
            package_path = get_package_share_directory('handeye_coord_transformer')
            config_path = os.path.join(package_path, 'config', 'EHMatrix.txt')
            
            # 读取16个浮点数
            EHArray = np.zeros(16, dtype=np.float32)
            
            with open(config_path, 'r') as file:
                # 读取所有以空格分隔的数字
                data = file.read().strip().split()
                
                if len(data) < 16:
                    self.get_logger().error(f"文件数据不足16个: {len(data)}")
                    raise ValueError("文件数据不足16个")
                
                for i in range(16):
                    EHArray[i] = float(data[i])
            
            self.get_logger().info(f"手眼矩阵加载成功")
            return EHArray
            
        except Exception as e:
            self.get_logger().error(f"加载标定文件失败: {e}")
            # 使用单位矩阵作为默认值
            return np.eye(4, dtype=np.float32).flatten()
    
    def analyze_eh_matrix(self):
        """分析手眼矩阵的性质"""
        # 将手眼矩阵重塑为4x4
        EH_matrix = np.array(self.EHArray).reshape(4, 4)
        
        self.get_logger().info("=== 手眼矩阵分析 ===")
        
        # 分离旋转和平移部分
        R = EH_matrix[:3, :3]
        T = EH_matrix[:3, 3]
        
        # 检查旋转矩阵的正交性
        RRT = R @ R.T
        identity = np.eye(3)
        ortho_error = np.max(np.abs(RRT - identity))
        
        self.get_logger().info(f"旋转矩阵正交性误差: {ortho_error:.6f}")
        
        # 检查行列式（应该是+1，表示纯旋转）
        det = np.linalg.det(R)
        self.get_logger().info(f"旋转矩阵行列式: {det:.6f}")
        
        # 平移向量
        self.get_logger().info(f"平移向量: [{T[0]:.1f}, {T[1]:.1f}, {T[2]:.1f}] mm")
        
        # 计算平移向量的范数
        translation_norm = np.linalg.norm(T)
        self.get_logger().info(f"平移向量长度: {translation_norm:.1f} mm")
        
        # 判断手眼矩阵的性质
        if translation_norm < 100:
            self.get_logger().info("手眼矩阵特征: 相机坐标系 → TCP坐标系 变换（眼在手中）")
        else:
            self.get_logger().info("手眼矩阵特征: 相机坐标系 → 基座坐标系 变换")
        
        # 打印完整的4x4矩阵
        self.get_logger().info("完整手眼矩阵:")
        for i in range(4):
            self.get_logger().info(f"  [{EH_matrix[i,0]:.6f}, {EH_matrix[i,1]:.6f}, "
                                  f"{EH_matrix[i,2]:.6f}, {EH_matrix[i,3]:.6f}]")
    
    def robot_state_callback(self, msg):
        """处理机器人状态的回调函数，提取TCP位姿"""
        try:
            if len(msg.machine_pose) >= 6:
                # 从消息中提取基座位姿
                # machine_pose格式: [x, y, z, rx, ry, rz]
                # 单位：毫米和弧
                x_mm = msg.machine_pose[0]  #  单位: 毫米
                y_mm = msg.machine_pose[1]   # 单位: 毫米
                z_mm = msg.machine_pose[2]  # 单位: 毫米
                rx_rad = msg.machine_pose[3]  # 单位: 弧度
                ry_rad = msg.machine_pose[4]  # 单位: 弧度
                rz_rad = msg.machine_pose[5]  # 单位: 弧度
                
                # 保存TCP位姿（这是TCP在基座坐标系下的位姿）
                self.current_tcp_pose = [x_mm, y_mm, z_mm, rx_rad, ry_rad, rz_rad]
                self.pose_received = True
                
                # 输出调试信息
                self.get_logger().debug(f"收到TCP位姿: [{x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}] mm")
                self.get_logger().debug(f"姿态欧拉角: [{rx_rad:.3f}, {ry_rad:.3f}, {rz_rad:.3f}] rad")
                
        except Exception as e:
            self.get_logger().error(f"处理机器人状态失败: {e}")
    
    def is_rotated_matrix(self, TcpTools):
        """验证旋转矩阵的有效性"""
        # 将9个元素的数组转换为3x3矩阵
        R = np.array(TcpTools).reshape(3, 3)
        
        # 计算R^T * R - I的范数
        RTR = R.T @ R
        I = np.eye(3)
        diff = RTR - I
        
        # 计算Frobenius范数的平方
        Norm = np.sum(diff**2)
        
        return Norm < 1e-6
    
    def ur_tcp_pose2matrix4f(self, ur_tcp_pose):
        """将TCP位姿转换为旋转矩阵"""
        if ur_tcp_pose is None or len(ur_tcp_pose) < 6:
            return None
        
        # 复制位姿数据
        ur_tcp_pose_tmp = ur_tcp_pose.copy()
        
        # 提取欧拉角（单位：弧度）
        rx = ur_tcp_pose_tmp[3]
        ry = ur_tcp_pose_tmp[4]
        rz = ur_tcp_pose_tmp[5]
        
        # 计算三角函数
        rxs = math.sin(rx)
        rxc = math.cos(rx)
        rys = math.sin(ry)
        ryc = math.cos(ry)
        rzs = math.sin(rz)
        rzc = math.cos(rz)
        
        # 构建绕X、Y、Z轴的旋转矩阵
        RotX = np.array([[1, 0, 0],
                         [0, rxc, -rxs],
                         [0, rxs, rxc]])
        
        RotY = np.array([[ryc, 0, rys],
                         [0, 1, 0],
                         [-rys, 0, ryc]])
        
        RotZ = np.array([[rzc, -rzs, 0],
                         [rzs, rzc, 0],
                         [0, 0, 1]])
        
        # 计算旋转矩阵：R = Rz * Ry * Rx（ZYX欧拉角顺序）
        Temp = RotY @ RotX
        TcpTools = RotZ @ Temp
        
        # 验证旋转矩阵的有效性
        if not self.is_rotated_matrix(TcpTools.flatten()):
            self.get_logger().warn("生成的旋转矩阵无效")
            return None
        
        return TcpTools.flatten()
    
    def transform_camera_to_base(self, pointx, pointy, pointz):
        """
        将相机坐标系下的点转换到基座坐标系
        假设手眼矩阵是：相机坐标系 → TCP坐标系
        机器人返回的TCP位姿是：TCP在基座坐标系下的位姿
        
        转换步骤：
        1. 相机点 → 手眼矩阵 → TCP坐标系下的点
        2. TCP坐标系下的点 → 基座坐标系（使用机器人返回的TCP位姿）
        """
        if self.current_tcp_pose is None:
            self.get_logger().warn("当前TCP位姿未收到")
            return None
        
        # 步骤1：将相机点转换到TCP坐标系
        # 使用手眼矩阵（相机→TCP）
        camera_point_homo = np.array([pointx, pointy, pointz, 1.0])
        EH_matrix = np.array(self.EHArray).reshape(4, 4)
        
        # 计算：P_tcp = EH × P_camera
        tcp_point_homo = EH_matrix @ camera_point_homo
        tcp_point = tcp_point_homo[:3]  # TCP坐标系下的点
        
        self.get_logger().info(f"TCP坐标系点: [{tcp_point[0]:.1f}, {tcp_point[1]:.1f}, {tcp_point[2]:.1f}] mm")
        
        # 步骤2：将TCP坐标系下的点转换到基座坐标系
        # 获取TCP在基座坐标系下的旋转矩阵
        R_base_to_tcp = self.ur_tcp_pose2matrix4f(self.current_tcp_pose)
        if R_base_to_tcp is None:
            self.get_logger().error("无法获取TCP旋转矩阵")
            return None
        
        # 将旋转矩阵重塑为3x3
        R_base_to_tcp = np.array(R_base_to_tcp).reshape(3, 3)
        
        # 我们需要将TCP坐标系下的点转换到基座坐标系
        # 公式：P_base = R_base_to_tcp × P_tcp + TCP_position
        # 其中R_base_to_tcp是基座坐标系到TCP坐标系的旋转矩阵
        tcp_position = np.array(self.current_tcp_pose[:3])
        
        # 计算基座坐标系下的点
        base_point = R_base_to_tcp @ tcp_point + tcp_position
        
        return base_point.tolist()
    
    def camera_point_callback(self, msg):
        """处理相机坐标的回调函数"""
        # 检查是否已收到TCP位姿
        if not self.pose_received or self.current_tcp_pose is None:
            self.get_logger().warn("尚未收到TCP位姿，无法进行坐标转换")
            return
        
        try:
            # 从消息中提取相机坐标（毫米单位）
            pointx = msg.center[0]
            pointy = msg.center[1]
            pointz = msg.center[2]
            
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"收到相机坐标: [{pointx:.1f}, {pointy:.1f}, {pointz:.1f}] mm")
            
            # 调用转换函数
            base_coords = self.transform_camera_to_base(pointx, pointy, pointz)
            
            if base_coords is None:
                self.get_logger().error("坐标转换失败")
                return
            
            # 解包结果
            basex, basey, basez = base_coords
            
            self.get_logger().info(f"基座坐标系坐标: [{basex:.1f}, {basey:.1f}, {basez:.1f}] mm")
            
            # 创建并发布目标位姿消息
            target_msg = PoseStamped()
            target_msg.header.stamp = self.get_clock().now().to_msg()
            target_msg.header.frame_id = "base_link"
            
            # 设置目标位置（基座坐标系）
            target_msg.pose.position.x = float(basex)
            target_msg.pose.position.y = float(basey)
            target_msg.pose.position.z = float(basez)
            
            # 将当前TCP的欧拉角转换为四元数
            current_rx = self.current_tcp_pose[3]
            current_ry = self.current_tcp_pose[4]
            current_rz = self.current_tcp_pose[5]
            quaternion = quaternion_from_euler(current_rx, current_ry, current_rz)
            
            # 设置目标姿态（使用当前TCP姿态）
            target_msg.pose.orientation.x = float(quaternion[0])
            target_msg.pose.orientation.y = float(quaternion[1])
            target_msg.pose.orientation.z = float(quaternion[2])
            target_msg.pose.orientation.w = float(quaternion[3])
            
            # 发布消息
            self.target_pub.publish(target_msg)
            
            # 输出转换结果
            self.get_logger().info("已发布目标位姿:")
            self.get_logger().info(f"  位置: [{target_msg.pose.position.x:.1f}, "
                                 f"{target_msg.pose.position.y:.1f}, "
                                 f"{target_msg.pose.position.z:.1f}] mm")
            self.get_logger().info(f"  姿态欧拉角: [{math.degrees(current_rx):.1f}, "
                                 f"{math.degrees(current_ry):.1f}, "
                                 f"{math.degrees(current_rz):.1f}]°")
            
            # 输出转换的完整信息
            self.log_conversion_summary(pointx, pointy, pointz, basex, basey, basez)
            
        except Exception as e:
            self.get_logger().error(f"坐标转换失败: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def log_conversion_summary(self, pointx, pointy, pointz, basex, basey, basez):
        """记录转换的完整信息"""
        # 获取当前TCP位置
        tcp_x = self.current_tcp_pose[0] 
        tcp_y = self.current_tcp_pose[1] 
        tcp_z = self.current_tcp_pose[2] 
        tcp_rx_deg = math.degrees(self.current_tcp_pose[3])
        tcp_ry_deg = math.degrees(self.current_tcp_pose[4])
        tcp_rz_deg = math.degrees(self.current_tcp_pose[5])
        
        # 计算偏移量
        offset_x = basex - tcp_x
        offset_y = basey - tcp_y
        offset_z = basez - tcp_z
        
        # 计算总偏移距离
        total_offset = math.sqrt(offset_x**2 + offset_y**2 + offset_z**2)
        
        self.get_logger().info("坐标转换汇总:")
        self.get_logger().info(f"相机坐标: [{pointx:.1f}, {pointy:.1f}, {pointz:.1f}] mm")
        self.get_logger().info(f"基座坐标: [{basex:.1f}, {basey:.1f}, {basez:.1f}] mm")
        self.get_logger().info(f"当前TCP:  [{tcp_x:.1f}, {tcp_y:.1f}, {tcp_z:.1f}] mm")
        self.get_logger().info(f"姿态:     [{tcp_rx_deg:.1f}, {tcp_ry_deg:.1f}, {tcp_rz_deg:.1f}] °")
        self.get_logger().info(f"X轴移动: {offset_x:+.1f} mm")
        self.get_logger().info(f"Y轴移动: {offset_y:+.1f} mm")
        self.get_logger().info(f"Z轴移动: {offset_z:+.1f} mm")
        self.get_logger().info(f"总移动距离: {total_offset:.1f} mm")
        self.get_logger().info("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeTransformNode()
    
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
