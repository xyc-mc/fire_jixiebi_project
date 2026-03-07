import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String
from base_msgs.msg import ControlMsg 
from elite_msgs.msg import RobotState
from unitreemotor_msgs.msg import MotorCommand, MotorStatus
from tf_transformations import quaternion_from_euler
import time
import numpy as np


class UninstallNode(Node):
    def __init__(self):
        super().__init__('uninstall_node')  # 拆卸节点
        
        self.uninstall_pose_stamped = None  # 拆卸目标位置信息
        self.current_robot_state = None  # 当前机器人状态信息
        self.pending_control_msg = None  # 待处理的控制指令
        self.move_completed = False  # 是否完成拆卸动作
        self.uninstall_frequency = 0  # 第几次扫描
        self.uninstall_class_name = None  # 类别名称
        self.first_move = False  # 是否第一次移动
        self.uninstall_move = False  # 是否开始拆卸
        self.complete_move = False  # 是否完成拆卸动作
        self.publish_count = 0  # 发布计数器
        self.max_publish_count = 3  # 最大发布次数
        self.is_second_publish = False  # 是否第二次发布
        self.is_down_move = False  # 是否下降
        self.is_back_move = False  # 是否返回原位
        self.publish_timer = None  # 用于发布控制指令的计时器
        self.step_timer = None  # 用于步骤控制的计时器
        self.execution_step = 0  # 执行步骤计数器
        
        # 新增：力矩调整相关变量（参考安装程序）
        self.is_adjusting = False  # 是否正在调整
        self.torque_check_timer = None
        self.last_torque_check_time = None
        self.torque_check_interval = 2
        self.z_adjustment = 0.0
        self.torque_condition_met = False
        
        # J2和J3关节的力矩参数
        self.last_j2_torque = None
        self.last_j3_torque = None
        self.current_j2_torque = None
        self.current_j3_torque = None
        
        # 力矩差值限制（可以根据拆卸需求调整）
        self.j2_max_diff = 1000.0
        self.j2_min_diff = 50.0
        self.j3_max_diff = 1000.0
        self.j3_min_diff = 80.0
        
        # 电机状态
        self.motor_status = None

        # 订阅目标位置信息
        self.subscription = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.listener_callback,
            10
        )  
        # 订阅电机状态信息
        self.motor_subscription = self.create_subscription(
            MotorStatus,
            'motor/status',
            self.motor_callback,
            10
        )
        # 订阅控制信息
        self.control_subscription = self.create_subscription(
            ControlMsg,
            '/sport_control',
            self.control_callback,
            10
        )
        # 订阅机械臂状态信息
        self.pose_subscription = self.create_subscription(
            RobotState,
            '/robot_state',
            self.current_callback,
            10
        )
        # 发布机械臂速度信息
        self.arm_velocity_publisher = self.create_publisher(Float64, '/arm_velocity', 10)
        # 发布电机控制指令
        self.motor_publisher = self.create_publisher(MotorCommand, 'motor/command', 10)
        # 发布目标位置信息
        self.publisher = self.create_publisher(PoseStamped, '/move_pose', 10)
        # 发布控制信息
        self.control_publisher = self.create_publisher(ControlMsg, '/move_control', 10)
        # 发布进度信息
        self.process_pub = self.create_publisher(String, '/process_data', 10)
        self.error_pub = self.create_publisher(String, '/error_data', 10)
        self.get_logger().info("Uninstall started.")
    
    def listener_callback(self, msg):
        try:
            if self.uninstall_frequency == 2 and self.uninstall_class_name == "head":
                if self.uninstall_pose_stamped is None:
                    self.uninstall_pose_stamped = msg
                    self.get_logger().info(f"保存卸载位置: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}")
                    
                    if self.pending_control_msg is not None:
                        self.get_logger().info("目标位置已收到，现在处理延迟的控制指令")
                        self.process_control_command(self.pending_control_msg)
                        self.pending_control_msg = None
        except Exception as e:
            self.get_logger().error(f"安装扫描保存错误: {e}")
    
    def current_callback(self, msg):
        try:
            self.current_robot_state = msg
            if self.is_adjusting and hasattr(msg, 'torque'):
                if len(msg.torque) >= 3:
                    self.current_j2_torque = msg.torque[1]
                    self.current_j3_torque = msg.torque[2]
            self.get_logger().debug(f"收到机器人状态，当前位置z={msg.machine_pose[2]:.3f}")
        except Exception as e:
            self.get_logger().error(f"Error in current_callback: {e}")
    
    def control_callback(self, msg):
        try:
            self.get_logger().info(f"收到控制指令: switch_control={msg.switch_control}, frequency={msg.frequency}")
            
            if msg.switch_control:
                self.uninstall_frequency = msg.frequency
                if hasattr(msg, 'class_name'):
                    self.uninstall_class_name = msg.class_name
                    self.get_logger().info(f"卸载种类: {msg.class_name}")
                
                if self.uninstall_pose_stamped is None and msg.frequency == 2:
                    self.get_logger().warn("还没有收到目标位置信息，保存控制指令等待处理")
                    self.pending_control_msg = msg
                    return
                
                self.process_control_command(msg)
                    
        except Exception as e:
            self.get_logger().error(f"Error in control_callback: {str(e)}")
            import traceback
            self.get_logger().error(f"详细错误信息:\n{traceback.format_exc()}")
    
    def motor_callback(self, msg):
        try:
            self.motor_status = msg
            self.get_logger().debug(f"收到电机状态，当前位置{msg.position}")
        except Exception as e:
            self.get_logger().error(f"Error in motor_callback: {e}")
    
    def check_torque_condition(self):
        """检查J2和J3关节的力矩差值条件"""
        if self.uninstall_frequency == 2 and self.uninstall_class_name == "head":
            if self.current_j2_torque is None or self.current_j3_torque is None:
                self.get_logger().warn("没有当前力矩值，无法检查条件")
                return False
            
            if self.last_j2_torque is None or self.last_j3_torque is None:
                self.last_j2_torque = self.current_j2_torque
                self.last_j3_torque = self.current_j3_torque
                self.get_logger().info("第一次力矩检查，保存初始力矩值")
                return False
            
            j2_diff = abs(self.current_j2_torque - self.last_j2_torque)
            j3_diff = abs(self.current_j3_torque - self.last_j3_torque)
            
            self.get_logger().info(f"力矩差值检查: J2差值={j2_diff:.3f}, J3差值={j3_diff:.3f}")
            
            j2_condition = self.j2_min_diff <= j2_diff <= self.j2_max_diff
            j3_condition = self.j3_min_diff <= j3_diff <= self.j3_max_diff
            
            if j2_condition and j3_condition:
                self.get_logger().info(f"力矩条件满足: J2差值{j2_diff:.3f}在范围[{self.j2_min_diff},{self.j2_max_diff}]内, "
                                    f"J3差值{j3_diff:.3f}在范围[{self.j3_min_diff},{self.j3_max_diff}]内")
                return True
            else:
                if not j2_condition:
                    self.get_logger().info(f"J2力矩条件不满足: {j2_diff:.3f}不在范围[{self.j2_min_diff},{self.j2_max_diff}]内")
                if not j3_condition:
                    self.get_logger().info(f"J3力矩条件不满足: {j3_diff:.3f}不在范围[{self.j3_min_diff},{self.j3_max_diff}]内")
                return False
    
    def torque_monitor_callback(self):
        """力矩监控定时器回调函数"""
        if self.uninstall_frequency == 2 and self.uninstall_class_name == "head":
            if not self.is_adjusting:
                return
                
            if self.check_torque_condition():
                self.torque_condition_met = True
                
                if self.torque_check_timer:
                    self.torque_check_timer.cancel()
                    self.torque_check_timer = None
                
                self.get_logger().info("J2和J3关节力矩条件满足，力矩调整完成")
                
                # 降低速度，准备下一步
                self.publish_velocity(5.0)
                
                # 进入下一步
                self.execution_step = 2.1
                if self.step_timer:
                    self.step_timer.cancel()
                self.step_timer = self.create_timer(2.0, self.execute_step)
            else:
                # 力矩条件不满足，继续调整Z轴
                self.z_adjustment += 0.8  # Z轴加1毫米
                self.get_logger().info(f"力矩条件不满足，调整Z轴: +{self.z_adjustment:.1f}")
                
                if self.current_j2_torque is not None:
                    self.last_j2_torque = self.current_j2_torque
                if self.current_j3_torque is not None:
                    self.last_j3_torque = self.current_j3_torque
                
                self.publish_move_command(first_move=False)
    
    def process_control_command(self, msg):
        """处理控制指令的通用方法"""
        try:
            if self.uninstall_frequency == 2 and self.uninstall_class_name == "head":
                if self.uninstall_pose_stamped is None:
                    self.get_logger().warn("无法处理控制指令: 没有目标位置信息")
                    return
                    
                if self.current_robot_state is None:
                    self.get_logger().warn("无法处理控制指令: 没有机器人状态信息")
                    return
                
                if not hasattr(self.current_robot_state, 'machine_pose'):
                    self.get_logger().warn("卸载失败: RobotState没有machine_pose属性")
                    return
                
                if len(self.current_robot_state.machine_pose) < 3:
                    self.get_logger().warn(f"卸载失败: machine_pose长度不足3，实际长度={len(self.current_robot_state.machine_pose)}")
                    return
                
                if not hasattr(self, 'motor_status'):
                    self.get_logger().warn("卸载失败: 电机状态未收到")
                    return
                
                # 重置状态
                self.execution_step = 0
                self.publish_count = 0
                self.is_second_publish = False
                self.is_down_move = False
                self.is_back_move = False
                
                # 重置力矩相关变量
                self.torque_condition_met = False
                self.z_adjustment = 0.0
                self.is_adjusting = False
                self.last_j2_torque = None
                self.last_j3_torque = None
                self.current_j2_torque = None
                self.current_j3_torque = None
                
                # 开始执行步骤1：第一次移动
                self.execute_step()
        except Exception as e:
            error = String()
            error.data = f"安装扫描保存错误: {str(e)}"
            self.error_pub.publish(error)

    def execute_step(self):
        """执行步骤控制"""
        try:
            process = String()
            process.data = 'Uninstall_move'
            self.process_pub.publish(process)
            
            if self.uninstall_frequency == 2 and self.uninstall_class_name == "head":
                # 取消之前的定时器
                if self.step_timer is not None:
                    self.step_timer.cancel()
                    self.step_timer = None
                    
                if self.execution_step == 0:
                    # 第一步：第一次移动（安全高度）
                    self.get_logger().info("步骤1: 执行第一次移动")
                    self.is_adjusting = False
                    self.publish_move_command(first_move=True)
                    self.publish_velocity(80.0)
                    self.execution_step = 1
                    # 8秒后执行第二步
                    self.step_timer = self.create_timer(8.0, self.execute_step)
                    
                elif self.execution_step == 1:
                    # 第二步：第二次移动（开始力矩调整）
                    self.get_logger().info("步骤2: 执行第二次移动，开始力矩调整")
                    self.is_second_publish = True
                    self.is_adjusting = True
                    self.is_down_move = False
                    self.is_back_move = False
                    
                    # 重置力矩相关变量
                    self.torque_condition_met = False
                    self.z_adjustment = 0.0
                    self.last_j2_torque = None
                    self.last_j3_torque = None
                    self.current_j2_torque = None
                    self.current_j3_torque = None
                    
                    # 发布初始位置
                    self.publish_move_command(first_move=False)
                    self.publish_velocity(8.0)
                    
                    # 启动力矩监控定时器（每2.5秒检查一次）
                    if self.torque_check_timer is None:
                        self.torque_check_timer = self.create_timer(self.torque_check_interval, self.torque_monitor_callback)
                        self.last_torque_check_time = time.time()
                        self.get_logger().info("开始力矩监控，每2.5秒检查一次J2和J3关节力矩差值")
                    
                    self.execution_step = 2
                    # 设置超时时间
                    self.step_timer = self.create_timer(60.0, self.execute_step)
                    
                elif self.execution_step == 2:
                    # 如果还在步骤2，说明力矩条件未满足或超时
                    if not self.torque_condition_met:
                        self.get_logger().warn("步骤2超时，力矩条件仍未满足，强制进入下一步")
                        self.is_adjusting = False
                        if self.torque_check_timer:
                            self.torque_check_timer.cancel()
                            self.torque_check_timer = None
                        
                        # 降低速度
                        self.publish_velocity(5.0)
                        
                        # 强制进入下一步
                        self.execution_step = 2.1
                        self.step_timer = self.create_timer(2.0, self.execute_step)
                        
                elif self.execution_step == 2.1:
                    # 力矩调整完成或超时，继续执行后续步骤
                    self.get_logger().info("步骤2.1: 力矩调整完成，继续后续步骤")
                    self.z_adjustment += 2
                    self.is_adjusting = False
                    self.is_second_publish = False
                    self.execution_step = 3
                    self.step_timer = self.create_timer(5.0, self.execute_step)
                    
                elif self.execution_step == 3:
                    # 第三步：旋转电机
                    self.get_logger().info("步骤3: 旋转电机")
                    self.is_second_publish = False
                    self.is_down_move = False
                    self.is_back_move = False
                    target_angle = self.motor_status.position - 1.5728
                    self.motor_rotate(target_angle)
                    self.execution_step = 4
                    # 等待10秒后执行第四步
                    self.step_timer = self.create_timer(10.0, self.execute_step)
                    
                elif self.execution_step == 4:
                    # 第四步：下降移动
                    self.get_logger().info("步骤4: 执行下降移动")
                    self.is_second_publish = False
                    self.is_down_move = True
                    self.is_back_move = False
                    self.publish_move_command(first_move=False)
                    self.publish_velocity(30.0)
                    self.execution_step = 5
                    # 等待5秒后执行第五步
                    self.step_timer = self.create_timer(8.0, self.execute_step)

                elif self.execution_step == 5:
                    # 第五步：回退移动
                    self.get_logger().info("步骤5: 执行回退移动")
                    self.is_second_publish = False
                    self.is_down_move = False
                    self.is_back_move = True
                    self.publish_move_command(first_move=False)
                    self.publish_velocity(80.0)
                    self.execution_step = 6
                    # 等待5秒后执行第六步
                    self.step_timer = self.create_timer(5.0, self.execute_step)
                    
                elif self.execution_step == 6:
                    # 第六步：完成
                    self.get_logger().info("步骤6: 卸载完成")
                    self.send_completion_control_msg()
                    self.reset_state()
                    self.motor_rotate(0.0)
        except Exception as e:
            error = String()
            error.data = f"安装扫描保存错误: {str(e)}"
            self.error_pub.publish(error)
     
    def publish_move_command(self, first_move=False):
        """发布移动命令"""
        if self.uninstall_pose_stamped is None:
            self.get_logger().warn("无法发布移动命令：没有目标位置信息")
            return
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        
        # 根据不同的步骤设置不同的偏移量
        if self.is_back_move:
            # 回退移动：返回安全位置
            pose_msg.pose.position.x = 450.0
            pose_msg.pose.position.y = 150.0
            pose_msg.pose.position.z = 380.0
            quat = quaternion_from_euler(0.0, 0.0, -2.87)
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            self.get_logger().info("回退移动: 返回安全位置")
            
        elif self.is_down_move:
            # 下降移动：向下移动
            z_offset = 200.0  # 下降轴
            pose_msg.pose.position.x = self.uninstall_pose_stamped.pose.position.x 
            pose_msg.pose.position.y = self.uninstall_pose_stamped.pose.position.y
            pose_msg.pose.position.z = self.uninstall_pose_stamped.pose.position.z - z_offset
            pose_msg.pose.orientation = self.uninstall_pose_stamped.pose.orientation
            self.get_logger().info(f"下降移动: z偏移-{z_offset}")
            
        elif self.is_second_publish:
            # 第二次移动：靠近目标，包含Z轴调整
            base_z_offset = 10  # 基础偏移量
            pose_msg.pose.position.x = self.uninstall_pose_stamped.pose.position.x
            pose_msg.pose.position.y = self.uninstall_pose_stamped.pose.position.y 
            # 应用Z轴调整
            pose_msg.pose.position.z = self.uninstall_pose_stamped.pose.position.z - base_z_offset + self.z_adjustment
            pose_msg.pose.orientation = self.uninstall_pose_stamped.pose.orientation
            self.get_logger().info(f"第二次移动: 基础z偏移-{base_z_offset}, 调整+{self.z_adjustment:.1f}")
            if self.torque_condition_met:
                self.get_logger().info(f"第二次移动(力矩调整完成): 基础z偏移-{base_z_offset}, 调整+{self.z_adjustment:.1f}")
            else:
                self.get_logger().info(f"第二次移动(调整中): 基础z偏移-{base_z_offset}, 调整+{self.z_adjustment:.1f}")
                
        elif first_move:
            # 第一次移动：安全高度
            z_offset = 60.0  # 安全高度
            pose_msg.pose.position.x = self.uninstall_pose_stamped.pose.position.x 
            pose_msg.pose.position.y = self.uninstall_pose_stamped.pose.position.y 
            pose_msg.pose.position.z = self.uninstall_pose_stamped.pose.position.z - z_offset 
            pose_msg.pose.orientation = self.uninstall_pose_stamped.pose.orientation
            self.get_logger().info(f"第一次移动: z偏移-{z_offset}")
        
        # 发布移动指令
        self.publisher.publish(pose_msg)
        self.publish_count += 1
        
        # 发布控制消息
        control_msg = ControlMsg()
        control_msg.frequency = self.uninstall_frequency + 1
        control_msg.switch_control = True
        self.control_publisher.publish(control_msg)
        self.get_logger().info(f"发布控制消息: frequency={control_msg.frequency}")
    
    def publish_velocity(self, velocity):
        """发布速度信息"""
        try:
            speed_msg = Float64()
            speed_msg.data = velocity
            self.arm_velocity_publisher.publish(speed_msg)
            self.get_logger().info(f"发布速度: {velocity}")
        except Exception as e:
            self.get_logger().error(f"发布速度失败: {e}")
    
    def motor_rotate(self, target_angle):
        """旋转电机"""
        try:
            if hasattr(self, 'motor_status'):                
                cmd = MotorCommand()
                cmd.command_type = 6        # 控制模式类型
                cmd.target_value = target_angle  # 目标角度
                cmd.kp = 0.098                # 比例增益
                cmd.kd = 0.024               # 微分增益
                cmd.tau = 0.06            # 力矩     
                self.motor_publisher.publish(cmd)
                self.get_logger().info(f"旋转电机到: {target_angle:.3f} 弧度，力矩: {cmd.tau}")
            else:
                self.get_logger().warn("电机状态未收到，无法旋转")
        except Exception as e:
            self.get_logger().error(f"电机旋转失败: {e}")
    
    def send_completion_control_msg(self):
        """发送完成控制消息"""
        next_control_msg = ControlMsg()
        next_control_msg.switch_control = True
        next_control_msg.frequency = self.uninstall_frequency + 1
        
        self.control_publisher.publish(next_control_msg)
        self.get_logger().info(f"卸载完成. 频次更新为: {next_control_msg.frequency}")
    
    def reset_state(self):
        """重置状态"""
        # 取消所有定时器
        if self.step_timer:
            self.step_timer.cancel()
            self.step_timer = None
        
        if self.publish_timer:
            self.publish_timer.cancel()
            self.publish_timer = None
        
        if self.torque_check_timer:
            self.torque_check_timer.cancel()
            self.torque_check_timer = None
        
        # 重置所有状态变量
        self.uninstall_pose_stamped = None
        self.current_robot_state = None
        self.pending_control_msg = None
        self.move_completed = False
        self.uninstall_frequency = 0
        self.uninstall_class_name = None
        self.first_move = False
        self.uninstall_move = False
        self.complete_move = False
        self.publish_count = 0
        self.is_second_publish = False
        self.is_down_move = False
        self.is_back_move = False
        self.execution_step = 0
        
        # 重置力矩相关变量
        self.is_adjusting = False
        self.torque_condition_met = False
        self.z_adjustment = 0.0
        self.last_j2_torque = None
        self.last_j3_torque = None
        self.current_j2_torque = None
        self.current_j3_torque = None
        
        self.get_logger().info("状态已重置，等待下次指令")

def main(args=None):
    rclpy.init(args=args)
    node = UninstallNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
