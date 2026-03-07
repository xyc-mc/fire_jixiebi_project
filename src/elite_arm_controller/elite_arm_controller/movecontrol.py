#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from elite_msgs.srv import InverseKinematic, AddPathPoint, ClearPathPoint, MoveByPath,StopMove
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState  
from std_msgs.msg import Float64,Bool
from elite_msgs.msg import RobotState
from base_msgs.msg import ControlMsg
import transforms3d as tfs
import math
import threading
import time

class PoseToJointController(Node):
    def __init__(self):
        super().__init__('pose_to_joint_controller')  
        self.get_logger().info("PoseToJointController节点已启动")      
        # 初始化变量
        self.initial_pose = False
        self.current_pose = None
        self.current_joints_rad = None
        self.current_joints_deg = None
        self.pose_received = False
        self.target_pose = None
        self.joint_received = False
        self.is_moving = False
        self.control_msg = ControlMsg()
        self.control_msg.complete = False
        self.lock = threading.Lock()

        
        # 创建订阅
        self.subscription = self.create_subscription(
            RobotState,
            '/robot_state',
            self.robot_state_callback,
            10
        )
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/move_pose',
            self.target_pose_callback,
            10
        )
        self.control_subscription = self.create_subscription(
            ControlMsg,
            '/move_control',
            self.control_callback,
            10
        )
        self.arm_velocity_subscription = self.create_subscription(
            Float64,
            '/arm_velocity',
            self.arm_velocity_callback,
            10
        )
        self.arm_stop_subscription = self.create_subscription(
            Bool,
            '/arm_stop',
            self.arm_stop_callback,
            10
        )
        # 创建发布
        self.move_complete_publisher = self.create_publisher(ControlMsg, 'camera_control', 10)
        # 创建服务客户端
        self.ik_client = self.create_client(InverseKinematic, 'inverse_kinematics')
        self.add_path_point_client = self.create_client(AddPathPoint, 'add_path_point_server')
        self.clear_path_point_client = self.create_client(ClearPathPoint, 'clear_path_point_server')
        self.move_by_path_client = self.create_client(MoveByPath, 'move_by_path_server')
        self.stop_move_client = self.create_client(StopMove, 'stop_move_server')
        # 等待服务可用
        self.wait_for_services()
        # 初始化定时器

    
    def wait_for_services(self):
        """等待服务可用"""
        try:
            self.get_logger().info("等待服务启动...")
            
            # 所有需要等待的服务
            services = [
                (self.ik_client, '逆运动学服务', 15.0),
                (self.add_path_point_client, '添加路径点服务', 10.0),
                (self.clear_path_point_client, '清除路径点服务', 10.0),
                (self.move_by_path_client, '执行路径服务', 10.0),
                (self.stop_move_client, '停止运动服务', 10.0)
            ]
            
            for client, name, timeout in services:
                self.get_logger().info(f"等待 {name} ...")
                if not client.wait_for_service(timeout_sec=timeout):
                    self.get_logger().error(f"{name} 等待超时")
                    return False
                self.get_logger().info(f"{name} 就绪")
            
            self.get_logger().info("所有服务已就绪")
            return True
            
        except Exception as e:
            self.get_logger().error(f"服务等待异常: {e}")
            return False
    
    def robot_state_callback(self, msg):
        """处理机器人状态消息"""
        try:
            with self.lock:
                if len(msg.machine_pose) >= 6:
                    x, y, z, rx, ry, rz = msg.machine_pose[:6]
                    self.current_pose = {
                        'position': [x, y, z],
                        'orientation': [rx, ry, rz]
                    }
                    if not self.pose_received:
                        self.pose_received = True
                        self.get_logger().info(f"首次接收到机器人位姿: {x:.2f}, {y:.2f}, {z:.2f}, {rx:.2f}, {ry:.2f}, {rz:.2f}")

        except Exception as e:
            self.get_logger().error(f'处理机器人状态时出错: {e}')
    def joint_state_callback(self, msg):
        """处理关节状态消息"""
        try:
            with self.lock:
                if len(msg.position) >= 6:
                    self.current_joints_rad = list(msg.position)[:6]
                    self.current_joints_deg = [math.degrees(j) for j in self.current_joints_rad]
                    if not self.joint_received:
                        self.joint_received = True
                        self.get_logger().info(f"首次接收到关节角度: {self.current_joints_deg}")
        except Exception as e:
            self.get_logger().error(f'处理关节状态时出错: {e}')
    
    def target_pose_callback(self, msg):
        """处理目标位姿消息"""
        if self.is_moving:
            return
        
        if not self.pose_received or not self.joint_received:
            return
        
        try:
            self.target_position_mm = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ]
            
            qx = msg.pose.orientation.x
            qy = msg.pose.orientation.y
            qz = msg.pose.orientation.z
            qw = msg.pose.orientation.w
            
            quat_norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
            
            if quat_norm < 1e-3 or (abs(qx) < 1e-3 and abs(qy) < 1e-3 and abs(qz) < 1e-3 and abs(qw - 1.0) < 1e-3):
                if self.current_pose is not None:
                    rx, ry, rz = self.current_pose['orientation']
                    quat = tfs.euler.euler2quat(rx, ry, rz, 'sxyz')
                else:
                    return
            else:
                quat = [qw, qx, qy, qz]
                if abs(quat_norm - 1.0) > 1e-3:
                    quat = [q / quat_norm for q in quat]
            
            euler_rad = tfs.euler.quat2euler(quat, 'sxyz')
            target_pose_array = self.target_position_mm + list(euler_rad)
            
            self.process_target_pose_async(target_pose_array)
            
        except Exception as e:
            self.get_logger().error(f'处理目标位姿时出错: {e}')   
    def arm_velocity_callback(self, msg):
        """处理速度消息"""
        try:
            self.arm_velocity = msg.data
        except Exception as e:
            self.get_logger().error(f'处理速度消息时出错: {e}')
    def arm_stop_callback(self, msg):
        """处理停止消息"""
        try:
            if msg.data:  # 只有当消息为True时才停止
                request = StopMove.Request()
                self.stop_move_client.call_async(request)
                self.get_logger().info("收到停止指令，正在停止机械臂...")
        except Exception as e:
            self.get_logger().error(f'处理停止消息时出错: {e}')
        
    def process_target_pose_async(self, target_pose_array):
        """异步处理目标位姿"""
        if self.is_moving:
            return
        
        self.is_moving = True
        
        request = InverseKinematic.Request()
        request.pose.position.x = float(target_pose_array[0])
        request.pose.position.y = float(target_pose_array[1])
        request.pose.position.z = float(target_pose_array[2])
        
        rx_rad, ry_rad, rz_rad = target_pose_array[3], target_pose_array[4], target_pose_array[5]
        quat = tfs.euler.euler2quat(rx_rad, ry_rad, rz_rad, 'sxyz')
        
        request.pose.orientation.w = float(quat[0])
        request.pose.orientation.x = float(quat[1])
        request.pose.orientation.y = float(quat[2])
        request.pose.orientation.z = float(quat[3])
        
        if self.current_joints_deg is None:
            self.is_moving = False
            return
        
        request.ref_joint = list(map(float, self.current_joints_deg))
        
        future = self.ik_client.call_async(request)
        future.add_done_callback(lambda future: self.ik_callback(future))
    
    def ik_callback(self, future):
        """逆运动学服务回调"""
        try:
            response = future.result()
            
            if response is None:
                self.is_moving = False
                return
            
            if response.result:
                target_joints_result = list(response.joint)
                target_joints_deg = target_joints_result
                
                thread = threading.Thread(
                    target=self.execute_path_move, 
                    args=(target_joints_deg,)
                )
                thread.start()
                
            else:
                self.is_moving = False
                
        except Exception as e:
            self.is_moving = False
    
    def execute_path_move(self, target_joints_deg):
        """执行路径运动"""
        try:
            if not self.clear_path_points():
                self.is_moving = False
                return
            
            if not self.add_path_point(target_joints_deg):
                self.is_moving = False
                return
            
            if not self.execute_move_by_path():
                self.is_moving = False
                return
            
            self.clear_path_points()
            
        except Exception as e:
            pass
        finally:
            self.is_moving = False
    
    def clear_path_points(self):
        """清除路径点"""
        try:
            request = ClearPathPoint.Request()
            future = self.clear_path_point_client.call_async(request)
            
            start_time = time.time()
            while time.time() - start_time < 2.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if future.done():
                    break
            
            if future.done():
                response = future.result()
                if response is not None and response.result:
                    return True
                else:
                    return False
            else:
                return False
            
        except Exception as e:
            return False
    
    def add_path_point(self, target_joints_deg):
        """添加路径点"""
        try:
            for i, angle in enumerate(target_joints_deg):
                if abs(angle) > 1000:
                    return False
            request = AddPathPoint.Request()
            request.way_point = list(map(float, target_joints_deg))
            
            request.move_type = 1
            request.speed = self.arm_velocity
            request.speed_type = 0
            request.acc = 20
            request.dec = 20
            request.smooth = 0
            request.circular_radius = 0
            
            request.cond_type = 0
            request.cond_num = 0
            request.cond_value = 0
            
            future = self.add_path_point_client.call_async(request)
            
            start_time = time.time()
            while time.time() - start_time < 2.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if future.done():
                    break
            
            if future.done():
                response = future.result()
                if response is not None and response.result:
                    return True
                else:
                    return False
            else:
                return False
            
        except Exception as e:
            return False
    
    def execute_move_by_path(self):
        """执行路径运动"""
        try:
            request = MoveByPath.Request()
            future = self.move_by_path_client.call_async(request)
            
            start_time = time.time()
            while time.time() - start_time < 20.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if future.done():
                    break

            if not future.done():
                return False
                
            response = future.result()
            
            if response is None or not response.result:
                return False
                
            tolerance = 2.0
            timeout = 30.0
            wait_start = time.time()

            
            while time.time() - wait_start < timeout:
                self.control_msg.complete = False
                current_position = self.current_pose.get('position', [0, 0, 0])
                
                target_x = self.target_position_mm[0]
                target_y = self.target_position_mm[1]
                target_z = self.target_position_mm[2]
                
                diff_x = abs(current_position[0] - target_x)
                diff_y = abs(current_position[1] - target_y)
                diff_z = abs(current_position[2] - target_z)
                

                # 如果位置达到容差或力矩超过阈值，则停止运动
                if (diff_x <= tolerance and diff_y <= tolerance and diff_z <= tolerance):
                    self.control_msg.complete = True
                    if self.control_msg.complete:   
                        self.camera_control_msg = ControlMsg()
                        self.camera_control_msg.switch_control = self.control_msg.switch_control
                        self.camera_control_msg.frequency = self.control_msg.frequency
                        self.camera_control_msg.complete = self.control_msg.complete
                        if self.control_msg.frequency <= 2:
                                self.move_complete_publisher.publish(self.camera_control_msg) 
                        return True
                time.sleep(0.1)
                rclpy.spin_once(self, timeout_sec=0.1)               
            return True               
        except Exception as e:
            return False
    
    def control_callback(self, msg):
        """处理控制消息"""
        try:           
            self.control_msg.switch_control = msg.switch_control
            self.control_msg.frequency = msg.frequency
        except Exception as e:
            pass
    


def main(args=None):
    rclpy.init(args=args)
    controller = PoseToJointController()
    
    try:
        # 等待所有服务就绪
        services_ready = controller.wait_for_services()
        if not services_ready:
            controller.get_logger().error("服务等待超时")
            return
        
        # 等待状态数据
        max_wait_time = 10.0  # 10秒超时
        start_time = time.time()
        
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0.1)
            
            if controller.pose_received and controller.joint_received:
                controller.get_logger().info("所有状态数据已接收，节点准备就绪")
                break
                
            if time.time() - start_time > max_wait_time:
                controller.get_logger().warn("状态数据等待超时，但节点将继续运行")
                break
        
        # 正常进入spin循环
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
