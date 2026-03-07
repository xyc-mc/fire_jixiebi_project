import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from base_msgs.msg import ControlMsg 
from elite_msgs.msg import RobotState
from unitreemotor_msgs.msg import MotorCommand, MotorStatus


class UninstallNode(Node):
    def __init__(self):
        super().__init__('uninstall_node')  # 拆卸节点
        
        self.uninstall_pose_stamped = None  # 拆卸目标位置信息
        self.current_robot_state = None # 当前机器人状态信息
        self.pending_control_msg = None # 待处理的控制指令
        self.move_completed = False # 是否完成拆卸动作
        self.uninstall_frequency = 0  # 第几次扫描
        self.uninstall_class_name = None # 类别名称
        self.first_move = False # 是否第一次移动
        self.uninstall_move = False # 是否开始拆卸
        self.complete_move = False # 是否完成拆卸动作
        self.publish_count = 0 # 发布计数器
        self.max_publish_count = 3 # 最大发布次数
        self.is_second_publish = False # 是否第二次发布
        self.is_back_move = False # 是否返回原位
        self.publish_timer = None # 用于发布控制指令的计时器
        self.step_timer = None  # 用于步骤控制的计时器
        self.execution_step = 0  # 执行步骤计数器
        
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
        # 发布电机控制指令
        self.motor_publisher = self.create_publisher(MotorCommand, 'motor/command', 10)
        # 发布目标位置信息
        self.publisher = self.create_publisher(PoseStamped, '/move_pose', 10)
        # 发布控制信息
        self.control_publisher = self.create_publisher(ControlMsg, '/move_control', 10)
        
        self.get_logger().info("Uninstall started.")
    
    def listener_callback(self, msg):
        try:
            if self.uninstall_frequency == 2:
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
    
    def process_control_command(self, msg):
        """处理控制指令的通用方法"""
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
            self.is_back_move = False
            
            # 开始执行步骤1：第一次移动
            self.execute_step()
    
    def execute_step(self):
        """执行步骤控制"""
        if self.step_timer is not None:
            self.step_timer.cancel()
            self.step_timer = None
            
        if self.execution_step == 0:
            # 第一步：第一次移动
            self.get_logger().info("步骤1: 执行第一次移动")
            self.publish_move_command(first_move=True)
            self.execution_step = 1
            # 8秒后执行第二步 - 修复：创建定时器但不取消
            self.step_timer = self.create_timer(8.0, self.execute_step)
            
        elif self.execution_step == 1:
            # 第二步：第二次移动
            self.get_logger().info("步骤2: 执行第二次移动")
            self.is_second_publish = True
            self.publish_move_command(first_move=False)
            self.execution_step = 2
            # 等待5秒后执行第三步 - 修复：创建定时器但不取消
            self.step_timer = self.create_timer(5.0, self.execute_step)
            
        elif self.execution_step == 2:
            # 第三步：旋转电机
            self.get_logger().info("步骤3: 旋转电机")
            self.motor_rotate()
            self.execution_step = 3
            # 等待2秒后执行第四步 - 修复：创建定时器但不取消
            self.step_timer = self.create_timer(10.0, self.execute_step)  # 注意：原代码是10秒，不是2秒
                
        elif self.execution_step == 3:
            # 第四步：回退移动（先下降轴再退50）
            self.get_logger().info("步骤4: 执行回退移动")
            self.is_back_move = True
            self.publish_move_command(first_move=False)
            self.execution_step = 4
            # 等待2秒后执行第五步 - 修复：创建定时器但不取消
            self.step_timer = self.create_timer(10.0, self.execute_step)  # 注意：原代码是10秒，不是2秒
                
        elif self.execution_step == 4:
            # 第五步：完成
            self.get_logger().info("步骤5: 卸载完成")
            self.send_completion_control_msg()
            self.reset_state()
    def publish_move_command(self, first_move=False):
        """发布移动命令"""
        if self.uninstall_pose_stamped is None:
            return
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        
        # 根据不同的步骤设置不同的偏移量
        if self.is_back_move:
            # 回退移动：先下降轴再退50
            z_offset = 300.0  # 下降轴
            x_offset = 50.0   # 后退50
            pose_msg.pose.position.x = self.uninstall_pose_stamped.pose.position.x + x_offset
            pose_msg.pose.position.y = self.uninstall_pose_stamped.pose.position.y
            pose_msg.pose.position.z = self.uninstall_pose_stamped.pose.position.z - z_offset
            self.get_logger().info(f"回退移动: x偏移+{x_offset}, z偏移-{z_offset}")
            
        elif self.is_second_publish:
            # 第二次移动
            z_offset = 10.0  # 靠近目标
            pose_msg.pose.position.x = self.uninstall_pose_stamped.pose.position.x
            pose_msg.pose.position.y = self.uninstall_pose_stamped.pose.position.y
            pose_msg.pose.position.z = self.uninstall_pose_stamped.pose.position.z - z_offset
            self.get_logger().info(f"第二次移动: z偏移-{z_offset}")
            
        elif first_move:
            # 第一次移动
            z_offset = 150.0  # 安全高度
            pose_msg.pose.position.x = self.uninstall_pose_stamped.pose.position.x
            pose_msg.pose.position.y = self.uninstall_pose_stamped.pose.position.y
            pose_msg.pose.position.z = self.uninstall_pose_stamped.pose.position.z - z_offset
            self.get_logger().info(f"第一次移动: z偏移-{z_offset}")
        
        # 使用目标位置的姿态
        pose_msg.pose.orientation = self.uninstall_pose_stamped.pose.orientation
        
        # 发布移动指令
        self.publisher.publish(pose_msg)
        self.publish_count += 1
        
        # 发布控制消息
        control_msg = ControlMsg()
        control_msg.frequency = self.uninstall_frequency + 1
        control_msg.switch_control = True
        self.control_publisher.publish(control_msg)
    
    def motor_rotate(self):
        """旋转电机"""
        try:
            if hasattr(self, 'motor_status'):
                # 计算目标角度（当前位置+60度，约1.047弧度）
                target_angle = self.motor_status.position + 1.047
                
                cmd = MotorCommand()
                cmd.command_type = 1        # 控制模式类型
                cmd.target_value = target_angle  # 目标角度
                cmd.kp = 0.89                # 比例增益
                cmd.kd = 0.12               # 微分增益
                
                self.motor_publisher.publish(cmd)
                self.get_logger().info(f"旋转电机到: {target_angle:.3f} 弧度")
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
        if self.step_timer:
            self.step_timer.cancel()
            self.step_timer = None
        
        if self.publish_timer:
            self.publish_timer.cancel()
            self.publish_timer = None
        
        # 重置所有状态变量 - 修复缩进
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
        self.is_back_move = False
        self.execution_step = 0
        
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