import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from base_msgs.msg import ControlMsg 
from elite_msgs.msg import RobotState
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math
import numpy as np

class SecondNode(Node):
    def __init__(self):
        super().__init__('second_node')

        self.first_pose_stamped = None  # 保存整个PoseStamped消息
        self.current_robot_state = None  # 保存RobotState消息
        self.pending_control_msg = None  # 保存等待处理的延迟控制指令
        self.move_completed = False
        self.install_frequency = 0
        self.install_class_name = None
        self.control_msg_angle = None
        self.quat_array = None

        # 订阅目标位置信息
        self.subscription = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.listener_callback,
            10
        )       
        # 订阅控制信息
        self.control_subscription = self.create_subscription(
            ControlMsg,
            '/sport_control',
            self.control_callback,
            10
        )
        self.pose_subscription = self.create_subscription(
            RobotState,
            '/robot_state',
            self.current_callback,
            10
        )
        # 发布目标位置信息
        self.publisher = self.create_publisher(PoseStamped, '/move_pose', 10)
        self.control_publisher = self.create_publisher(ControlMsg, '/move_control', 10)
        self.get_logger().info("second_scan Started.")
    
    def listener_callback(self, msg):
        try:
            # 保存第一个收到的位姿
            if self.first_pose_stamped is None:
                self.first_pose_stamped = msg
                self.get_logger().info(f"第二次扫描保存到: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}")
                
                # 如果有等待处理的延迟控制指令，现在处理它
                if self.pending_control_msg is not None:
                    self.get_logger().info("目标位置已收到，现在处理延迟的控制指令")
                    self.process_control_command(self.pending_control_msg)
                    self.pending_control_msg = None
        except Exception as e:
            self.get_logger().error(f"第二次扫描保存错误: {e}")
    
    def current_callback(self, msg):
        try:
            # 保存RobotState消息
            self.current_robot_state = msg
            self.get_logger().debug(f"收到机器人状态，当前位置z={msg.machine_pose[2]:.3f}")
        except Exception as e:
            self.get_logger().error(f"Error in current_callback: {e}")
    
    def control_callback(self, msg):
        try:
            self.get_logger().info(f"收到控制指令: switch_control={msg.switch_control}, frequency={msg.frequency}")
            
            # 检查是否启用控制
            if msg.switch_control:
                # 保存控制指令的参数
                self.install_frequency = msg.frequency
                self.control_msg_angle = msg.angle  
                # 如果还没有收到目标位置信息，保存控制指令等待
                if self.first_pose_stamped is None and msg.frequency == 2:
                    self.get_logger().warn("还没有收到目标位置信息，保存控制指令等待处理")
                    self.pending_control_msg = msg
                    return
                
                # 处理控制指令
                self.process_control_command(msg)
                    
        except Exception as e:
            self.get_logger().error(f"Error in control_callback: {str(e)}")
            import traceback
            self.get_logger().error(f"详细错误信息:\n{traceback.format_exc()}")
    
    def process_control_command(self, msg):
        """处理控制指令的通用方法"""
        # 检查频率是否为2
        if self.install_frequency == 2:
            if self.first_pose_stamped is None:
                self.get_logger().warn("无法处理控制指令: 没有目标位置信息")
                return
                
            if self.current_robot_state is None:
                self.get_logger().warn("无法处理控制指令: 没有机器人状态信息")
                return
            
            # 检查是否有machine_pose属性
            if not hasattr(self.current_robot_state, 'machine_pose'):
                self.get_logger().warn("第二次扫描失败: RobotState没有machine_pose属性")
                return
            
            # 检查machine_pose长度
            if len(self.current_robot_state.machine_pose) < 3:
                self.get_logger().warn(f"第二次扫描失败: machine_pose长度不足3，实际长度={len(self.current_robot_state.machine_pose)}")
                return
            
            # 创建新的PoseStamped消息
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'world'
            
            # 设置位置：使用第一个位姿的x和y，使用当前机器人的z
            pose_msg.pose.position.x = self.first_pose_stamped.pose.position.x - 98.963
            pose_msg.pose.position.y = self.first_pose_stamped.pose.position.y - 21.52
            pose_msg.pose.position.z = self.current_robot_state.machine_pose[2]
            
            # 计算四元数
            self.Normal_vector_to_angle()
            
            # 设置四元数（使用正确的属性访问方式）
            pose_msg.pose.orientation.x = self.quat_array[0]
            pose_msg.pose.orientation.y = self.quat_array[1]
            pose_msg.pose.orientation.z = self.quat_array[2]
            pose_msg.pose.orientation.w = self.quat_array[3]
            
            # 发布移动指令
            self.publisher.publish(pose_msg)
            self.get_logger().info(f"发布移动指令: x={pose_msg.pose.position.x:.3f}, y={pose_msg.pose.position.y:.3f}, z={pose_msg.pose.position.z:.3f}")
            
            # 修复：将四元数转换为欧拉角显示
            # 方法1：使用四元数属性
            self.get_logger().info(f"发布四元数指令: x={pose_msg.pose.orientation.x:.3f}, y={pose_msg.pose.orientation.y:.3f}, "
                                 f"z={pose_msg.pose.orientation.z:.3f}, w={pose_msg.pose.orientation.w:.3f}")
            
            # 方法2：转换为欧拉角显示（可选）
            quat_array = [
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w
            ]
            rx, ry, rz = euler_from_quaternion(quat_array)
            self.get_logger().info(f"发布欧拉角指令: rx={rx:.3f}, ry={ry:.3f}, rz={rz:.3f}")
            
            self.first_frequency = msg.frequency
        
            # 创建并发布下一个频率的控制消息
            next_control_msg = ControlMsg()
            next_control_msg.switch_control = True  # 保持启用状态
            next_control_msg.frequency = msg.frequency + 1
            
            self.control_publisher.publish(next_control_msg)
            self.get_logger().info(f"第二次扫描完成. 频次增加={next_control_msg.frequency}")
            
            # 重置以便下次重新保存
            self.first_pose_stamped = None
            self.move_completed = False

    def Normal_vector_to_angle(self):
        """
        根据目标Z轴方向向量计算Rx和Ry角          
        返回:
            rx, ry: 弧度制的旋转角度
        """
        # 获取向量分量
        nx = self.control_msg_angle[0] 
        ny = self.control_msg_angle[1]
        nz = self.control_msg_angle[2]
        
        # 归一化向量
        norm = math.sqrt(nx**2 + ny**2 + nz**2)
        if norm < 1e-6:
            self.get_logger().warn("向量太小，无法归一化")
            return 0.0, 0.0
            
        nx_norm = nx / norm
        ny_norm = ny / norm
        nz_norm = nz / norm
        
        # 计算角度
        if abs(abs(ny_norm) - 1.0) < 1e-6:
            rx1 = -math.pi/2 if ny_norm > 0 else math.pi/2
            ry1 = 0.0
        else:
            rx1 = math.asin(-ny_norm)  # 注意：负号
            ry1 = math.atan2(nx_norm, nz_norm)
            
        # 计算相对于当前姿态的增量
        rx = self.current_robot_state.machine_pose[3] - ry1
        ry = self.current_robot_state.machine_pose[4] + rx1
        rz = self.current_robot_state.machine_pose[5]
        
        # 转换为四元数
        self.quat_array = quaternion_from_euler(rx, ry, rz)  
        self.get_logger().info(f"旋转角度(弧度): rx={rx:.3f}, ry={ry:.3f}, rz={rz:.3f}")
        
        return rx, ry

def main(args=None):
    rclpy.init(args=args)
    node = SecondNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()