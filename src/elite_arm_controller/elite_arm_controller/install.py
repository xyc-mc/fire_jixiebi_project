import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Float64, Bool, String
from base_msgs.msg import ControlMsg 
from elite_msgs.msg import RobotState
from unitreemotor_msgs.msg import MotorCommand, MotorStatus
from tf_transformations import quaternion_from_euler
import time
import math
import numpy as np
from enum import Enum, auto
from typing import Optional, List, Tuple


# ============================================
# 状态枚举定义
# ============================================
class InstallationState(Enum):
    """安装过程的状态枚举"""
    IDLE = auto()
    INITIALIZING = auto()
    MOVING_TO_SAFE_HEIGHT = auto()
    ROTATING_MOTOR_FOR_ALIGNMENT = auto()
    APPROACHING_TARGET = auto()
    TORQUE_ADJUSTMENT = auto()
    MOTOR_FINE_TUNING = auto()
    POST_ALIGNMENT_DELAY = auto()
    POST_ALIGNMENT_ADJUSTMENT = auto()
    EXECUTING_INSTALLATION = auto()
    MOVING_DOWN = auto()
    RETRACTING = auto()
    COMPLETED = auto()
    ERROR = auto()


# ============================================
# 配置类
# ============================================
class InstallationConfig:
    """安装过程的配置参数"""
    # 力矩调整参数
    TORQUE_CHECK_INTERVAL = 2.0  # 秒
    J2_MAX_TORQUE_DIFF = 1000.0
    J2_MIN_TORQUE_DIFF = 50.0
    J3_MAX_TORQUE_DIFF = 1000.0
    J3_MIN_TORQUE_DIFF = 75.0
    
    # Z轴调整参数
    Z_ADJUST_INCREMENT = 1.0  # mm - 力矩调整阶段的Z轴调整增量
    Z_FINE_TUNING_INCREMENT = 0.2  # mm - 精调阶段每次序列切换时Z轴调整增量
    Z_POST_ALIGNMENT_ADJUSTMENT = 3.0  # mm - 对准检测后的Z轴调整
    
    # 延时参数
    POST_ALIGNMENT_DELAY_TIME = 10.0  # 秒 - 对准后延时时间
    AXIAL_ADJUSTMENT_DELAY_TIME = 5.0  # 秒 - 轴向调整后延时时间
    MOTOR_SETTLE_TIME = 2.0  # 秒 - 电机到位后稳定时间
    FINE_TUNING_DELAY = 1.0  # 秒 - 精调摆动间隔
    
    # 电机精调参数 - 基于序列的摆动策略
    FIXED_SWING_ANGLE = 0.3  # 弧度 - 固定的摆动角度
    MAX_SEQUENCE_COUNT = 60  # 最大序列次数
    INITIAL_SEQUENCE = 1  # 起始序列次数
    
    # 电机控制参数
    MOTOR_POSITION_TOLERANCE = 0.15  # 弧度 - 电机位置容差
    MOTOR_WAIT_TIMEOUT = 10.0  # 秒 - 等待电机到位超时时间
    
    # 摆动控制参数
    MAX_FINE_TUNING_STEPS = 465  # 最大精调步骤（60序列的总和：1+2+...+60=1830，实际用465步）
    TORQUE_DIFF_THRESHOLD = 80.0
    REQUIRED_STABLE_COUNT = 3
    FINE_TUNING_TIMEOUT = 120.0  # 电机精调超时时间
    
    # 安装移动参数
    SAFE_HEIGHT_OFFSET = 75.0  # mm
    APPROACH_HEIGHT_OFFSET = 45.0  # mm
    DOWN_MOVE_OFFSET = 200.0  # mm
    INSTALLATION_ROTATION = 1.5782  # 弧度
    
    # 安全位置
    SAFE_POSITION = (450.0, 150.0, 380.0)  # (x, y, z) in mm
    SAFE_ORIENTATION_EULER = (0.0, 0.0, -2.97)  # 弧度
    
    # 速度设置
    HIGH_SPEED = 80.0
    MEDIUM_SPEED = 20.0
    LOW_SPEED = 10.0
    VERY_LOW_SPEED = 5.0
    
    # 电机控制增益
    MOTOR_KP = 0.482  # 增加刚度
    MOTOR_KD = 0.215
    MOTOR_TAU = 0.08


# ============================================
# 数据容器类
# ============================================
class TorqueData:
    """力矩数据容器"""
    def __init__(self):
        self.j2_torque: Optional[float] = None
        self.j3_torque: Optional[float] = None
        self.last_j2_torque: Optional[float] = None
        self.last_j3_torque: Optional[float] = None
        self.torque_history: List[float] = []
    
    def update_current(self, j2: float, j3: float):
        """更新当前力矩值"""
        self.j2_torque = j2
        self.j3_torque = j3
    
    def update_history(self):
        """将当前力矩差值保存到历史记录"""
        if self.j2_torque is not None and self.j3_torque is not None:
            torque_diff = abs(self.j2_torque - self.j3_torque)
            self.torque_history.append(torque_diff)
            if len(self.torque_history) > 10:
                self.torque_history.pop(0)
    
    def reset(self):
        """重置所有力矩数据"""
        self.j2_torque = None
        self.j3_torque = None
        self.last_j2_torque = None
        self.last_j3_torque = None
        self.torque_history.clear()


class FineTuningData:
    """精调数据容器 - 修改版：基于序列的摆动策略"""
    def __init__(self):
        self.step_count = 0  # 总摆动次数
        self.sequence_number = 1  # 当前序列号
        self.sequence_count = 0  # 当前序列已执行次数
        self.sequence_length = InstallationConfig.INITIAL_SEQUENCE  # 当前序列长度
        
        # 当前摆动方向：1=正转（顺时针），-1=反转（逆时针）
        self.direction = 1  # 起始为正转（奇数序列）
        
        # 位置信息
        self.motor_center_position: float = 0.0  # 初始中心位置
        self.current_position: float = 0.0  # 当前电机位置
        self.target_position: float = 0.0  # 目标电机位置
        self.alignment_position: float = 0.0  # 检测到对准时的位置
        self.torque_stable_count = 0
        
        # 状态标志
        self.alignment_detected = False
        self.completed = False
        self.stop_flag = False
        self.waiting_for_motor = False  # 等待电机到位标志
        
        # Z轴调整累计
        self.z_adjustment_total = 0.0  # 毫米
        
        # 摆动历史记录
        self.swing_history = []
    
    def get_next_direction(self):
        """获取下一个摆动方向：奇数序列为正转，偶数序列为反转"""
        return 1 if self.sequence_number % 2 == 1 else -1
    
    def is_sequence_complete(self):
        """检查当前序列是否完成"""
        return self.sequence_count >= self.sequence_length
    
    def prepare_next_sequence(self):
        """准备下一个序列"""
        self.sequence_number += 1
        self.sequence_count = 0
        self.sequence_length += 1  # 每个序列增加一次摆动次数
        self.direction = self.get_next_direction()  # 切换方向
        
        # 检查是否达到最大序列次数
        if self.sequence_number > InstallationConfig.MAX_SEQUENCE_COUNT:
            return False
        return True


class InstallationData:
    """安装过程数据容器"""
    def __init__(self):
        self.target_pose: Optional[PoseStamped] = None
        self.current_robot_state: Optional[RobotState] = None
        self.motor_status: Optional[MotorStatus] = None
        self.pending_control_msg: Optional[ControlMsg] = None
        
        # 安装参数
        self.install_frequency = 0
        self.install_class_name: Optional[str] = None
        self.match_angle: float = 0.0
        self.target_orientation = Quaternion()
        
        # 调整参数
        self.z_adjustment: float = 0.0  # 力矩调整阶段的Z轴累积调整
        self.z_fine_adjustment: float = 0.0  # 精调阶段的Z轴调整（累积调整）
        self.z_post_torque_adjustment: float = 0.0  # 力矩条件满足后的Z轴调整
        self.is_adjusting = False
        
        # 状态标志
        self.move_completed = False
        self.install_completed = False
        self.torque_condition_met = False


# ============================================
# 主节点类
# ============================================
class InstallationNode(Node):
    """机械臂安装控制节点"""
    
    def __init__(self):
        super().__init__('install_node')
        
        # 初始化配置和数据容器
        self.config = InstallationConfig()
        self.installation_data = InstallationData()
        self.torque_data = TorqueData()
        self.fine_tuning_data = FineTuningData()
        
        # 状态管理
        self.current_state = InstallationState.IDLE
        
        # 定时器
        self.step_timer = None
        self.torque_check_timer = None
        self.fine_tuning_timeout_timer = None
        self.motor_wait_timer = None  # 等待电机定时器
        self.fine_tuning_monitor_timer = None  # 精调监控定时器
        
        # 发布器和订阅器
        self._initialize_publishers()
        self._initialize_subscribers()
        
        self.get_logger().info("Install Started.")
    
    def _initialize_publishers(self):
        """初始化所有发布器"""
        self.arm_velocity_pub = self.create_publisher(Float64, '/arm_velocity', 10)
        self.arm_stop_pub = self.create_publisher(Bool, '/arm_stop', 10)
        self.motor_cmd_pub = self.create_publisher(MotorCommand, 'motor/command', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, '/move_pose', 10)
        self.control_pub = self.create_publisher(ControlMsg, '/move_control', 10)
        self.process_pub = self.create_publisher(String, '/process_data', 10)
        self.error_pub = self.create_publisher(String, '/error_data', 10)
    
    def _initialize_subscribers(self):
        """初始化所有订阅器"""
        self.create_subscription(
            PoseStamped,
            '/target_pose',
            self._target_pose_callback,
            10
        )
        self.create_subscription(
            MotorStatus,
            'motor/status',
            self._motor_status_callback,
            10
        )
        self.create_subscription(
            ControlMsg,
            '/sport_control',
            self._control_msg_callback,
            10
        )
        self.create_subscription(
            RobotState,
            '/robot_state',
            self._robot_state_callback,
            10
        )
    
    # ============================================
    # 回调函数
    # ============================================
    
    def _target_pose_callback(self, msg: PoseStamped):
        """目标位置回调函数"""
        try:
            if self.installation_data.install_frequency == 2:
                if self.installation_data.target_pose is None:
                    self.installation_data.target_pose = msg
                    self._log_pose_info("安装扫描保存到", msg.pose.position)
                    
                    if self.installation_data.pending_control_msg is not None:
                        self.get_logger().info("目标位置已收到，处理延迟的控制指令")
                        self._process_control_command(self.installation_data.pending_control_msg)
                        self.installation_data.pending_control_msg = None
        except Exception as e:
            self.get_logger().error(f"安装扫描保存错误: {e}")
    
    def _robot_state_callback(self, msg: RobotState):
        """机器人状态回调函数"""
        try:
            self.installation_data.current_robot_state = msg
            if self.installation_data.is_adjusting and hasattr(msg, 'torque'):
                if len(msg.torque) >= 3:
                    self.torque_data.update_current(msg.torque[1], msg.torque[2])
        except Exception as e:
            self.get_logger().error(f"Error in current_callback: {e}")
    
    def _motor_status_callback(self, msg: MotorStatus):
        """电机状态回调函数 - 修改版：支持位置到位检测"""
        try:
            self.installation_data.motor_status = msg
            
            # 更新电机当前位置
            current_position = msg.position
            self.fine_tuning_data.current_position = current_position
            
            # 在精调开始时更新电机中心位置
            if (self.current_state == InstallationState.MOTOR_FINE_TUNING and 
                self.fine_tuning_data.step_count == 0 and 
                self.fine_tuning_data.motor_center_position == 0.0):
                self.fine_tuning_data.motor_center_position = current_position
                self.get_logger().info(f"设置电机中心位置: {current_position:.3f}弧度")
            
            # 检查是否在等待电机到位
            if self.fine_tuning_data.waiting_for_motor and self.fine_tuning_data.target_position != 0.0:
                # 检查电机是否到达目标位置
                target_position = self.fine_tuning_data.target_position
                position_error = abs(current_position - target_position)
                
                if position_error <= self.config.MOTOR_POSITION_TOLERANCE:
                    # 电机到位，重置等待标志
                    self.fine_tuning_data.waiting_for_motor = False
                    self.get_logger().info(f"电机已到位: 当前={current_position:.3f}, 目标={target_position:.3f}")
                    
                    # 停止等待定时器
                    if self.motor_wait_timer:
                        self.motor_wait_timer.cancel()
                        self.motor_wait_timer = None
                    
                    # 继续执行精调流程
                    self._continue_fine_tuning_after_motor_move()
                        
        except Exception as e:
            self.get_logger().error(f"Error in motor_callback: {e}")
    
    def _control_msg_callback(self, msg: ControlMsg):
        """控制指令回调函数"""
        try:
            self.get_logger().info(f"收到控制指令: switch_control={msg.switch_control}, frequency={msg.frequency}")
            
            if msg.switch_control:
                self.installation_data.install_frequency = msg.frequency
                
                # 保存附加信息
                if hasattr(msg, 'class_name'):
                    self.installation_data.install_class_name = msg.class_name
                    self.get_logger().info(f"扫描种类: {msg.class_name}")
                
                if hasattr(msg, 'rotate_angle'):
                    self.installation_data.match_angle = math.radians(msg.rotate_angle)
                
                # 检查是否准备好处理指令
                if not self._is_ready_for_control_command(msg):
                    self.installation_data.pending_control_msg = msg
                    return
                
                self._process_control_command(msg)
                
        except Exception as e:
            self.get_logger().error(f"Error in control_callback: {str(e)}")
            import traceback
            self.get_logger().error(f"详细错误信息:\n{traceback.format_exc()}")
    
    # ============================================
    # 控制逻辑
    # ============================================
    
    def _is_ready_for_control_command(self, msg: ControlMsg) -> bool:
        """检查是否准备好处理控制指令"""
        if msg.frequency != 2:
            return True
        
        if self.installation_data.target_pose is None:
            self.get_logger().warn("还没有收到目标位置信息，保存控制指令等待处理")
            return False
        
        if self.installation_data.current_robot_state is None:
            self.get_logger().warn("无法处理控制指令: 没有机器人状态信息")
            return False
        
        if not hasattr(self.installation_data.current_robot_state, 'machine_pose'):
            self.get_logger().warn("安装扫描失败: RobotState没有machine_pose属性")
            return False
        
        if len(self.installation_data.current_robot_state.machine_pose) < 3:
            self.get_logger().warn(f"安装扫描失败: machine_pose长度不足3")
            return False
        
        if self.installation_data.motor_status is None:
            self.get_logger().warn("安装扫描失败: 电机状态未收到")
            return False
        
        return True
    
    def _process_control_command(self, msg: ControlMsg):
        """处理控制指令"""
        try:
            if self._should_process_installation():
                self._reset_installation_state()
                self._start_installation_procedure()
            else:
                self.get_logger().warn(f"控制指令不符合安装条件")
                
        except Exception as e:
            error_msg = String()
            error_msg.data = f"处理控制指令失败: {str(e)}"
            self.error_pub.publish(error_msg)
    
    def _should_process_installation(self) -> bool:
        """检查是否应该处理安装指令"""
        return (self.installation_data.install_frequency == 2 and 
                self.installation_data.install_class_name == "base")
    
    # ============================================
    # 状态管理
    # ============================================
    
    def _reset_installation_state(self):
        """重置安装状态"""
        self._stop_all_timers()
        
        # 重置状态
        self.current_state = InstallationState.INITIALIZING
        
        # 重置数据
        self.installation_data.move_completed = False
        self.installation_data.install_completed = False
        self.installation_data.torque_condition_met = False
        self.installation_data.z_adjustment = 0.0
        self.installation_data.z_fine_adjustment = 0.0
        self.installation_data.z_post_torque_adjustment = 0.0
        self.installation_data.is_adjusting = False
        
        self.torque_data.reset()
        
        self.fine_tuning_data = FineTuningData()
        
        self.get_logger().info("安装状态已重置")
    
    def _start_installation_procedure(self):
        """开始安装流程"""
        self._transition_to_state(InstallationState.MOVING_TO_SAFE_HEIGHT)
    
    def _transition_to_state(self, new_state: InstallationState):
        """状态转换"""
        self.get_logger().info(f"状态转换: {self.current_state.name} -> {new_state.name}")
        self.current_state = new_state
        
        # 执行新状态对应的操作
        self._execute_current_state()
    
    def _execute_current_state(self):
        """执行当前状态对应的操作"""
        self._publish_process_message('Install_move')
        
        # 根据状态执行不同的操作
        state_handlers = {
            InstallationState.MOVING_TO_SAFE_HEIGHT: self._execute_move_to_safe_height,
            InstallationState.ROTATING_MOTOR_FOR_ALIGNMENT: self._execute_motor_alignment,
            InstallationState.APPROACHING_TARGET: self._execute_approach_target,
            InstallationState.TORQUE_ADJUSTMENT: self._execute_torque_adjustment,
            InstallationState.MOTOR_FINE_TUNING: self._execute_motor_fine_tuning,
            InstallationState.POST_ALIGNMENT_DELAY: self._execute_post_alignment_delay,
            InstallationState.POST_ALIGNMENT_ADJUSTMENT: self._execute_post_alignment_adjustment,
            InstallationState.EXECUTING_INSTALLATION: self._execute_installation,
            InstallationState.MOVING_DOWN: self._execute_move_down,
            InstallationState.RETRACTING: self._execute_retract,
            InstallationState.COMPLETED: self._execute_completion,
        }
        
        handler = state_handlers.get(self.current_state)
        if handler:
            handler()
    
    # ============================================
    # 状态处理函数
    # ============================================
    
    def _execute_move_to_safe_height(self):
        """执行移动到安全高度"""
        self.get_logger().info("步骤1: 执行第一次移动")
        self._set_arm_velocity(self.config.HIGH_SPEED)
        self._publish_move_command(is_safe_height=True)
        
        # 设置超时定时器
        self._start_step_timer(8.0, InstallationState.ROTATING_MOTOR_FOR_ALIGNMENT)
    
    def _execute_motor_alignment(self):
        """执行电机角度匹配"""
        self.get_logger().info("步骤2: 旋转匹配角度")
        
        if self.installation_data.motor_status:
            target_angle = self.installation_data.motor_status.position + self.installation_data.match_angle
            self._rotate_motor(target_angle)
        
        self._start_step_timer(5.0, InstallationState.APPROACHING_TARGET)
    
    def _execute_approach_target(self):
        """执行接近目标"""
        self.get_logger().info("步骤3: 执行第二次移动，开始机械臂力矩调整")
        
        self.installation_data.is_adjusting = True
        self._set_arm_velocity(self.config.LOW_SPEED)
        self._publish_move_command(is_approach=True)
        
        # 启动力矩监控
        self._start_torque_monitoring()
        
        # 设置超时
        self._check_and_handle_torque_immediately()
        
        # 设置超时定时器
        self._start_step_timer(30.0, InstallationState.TORQUE_ADJUSTMENT)
    
    def _check_and_handle_torque_immediately(self):
        """立即检查力矩条件并处理"""
        if self._check_torque_condition():
            self.installation_data.torque_condition_met = True
            self._stop_torque_monitoring()
            self._stop_step_timer()
            self._set_arm_velocity(self.config.VERY_LOW_SPEED)
            
            # 力矩条件满足后，不再直接提升Z轴
            self.installation_data.z_post_torque_adjustment = 0.0
            
            # 立即开始电机精调
            self._start_step_timer(1.0, InstallationState.MOTOR_FINE_TUNING)
    
    def _execute_torque_adjustment(self):
        """执行力矩调整（超时处理）"""
        if self._check_torque_condition():
            self.get_logger().info("力矩条件满足，进入电机精调")
            self._set_arm_velocity(self.config.VERY_LOW_SPEED)
            
            # 力矩条件满足后，不再直接提升Z轴
            self.installation_data.z_post_torque_adjustment = 0.0
            
            # 直接进入电机精调
            self._start_step_timer(0.5, InstallationState.MOTOR_FINE_TUNING)
        else:
            self.get_logger().warn("力矩调整超时，强制进入电机精调")
            self.installation_data.is_adjusting = False
            self._stop_torque_monitoring()
            self._transition_to_state(InstallationState.MOTOR_FINE_TUNING)
    
    def _execute_motor_fine_tuning(self):
        """执行电机精调"""
        self.get_logger().info("步骤3.1: 机械臂力矩调整完成，开始电机精调试探")
        
        self.installation_data.is_adjusting = True
        
        # 重置精调阶段的Z轴调整量，从0开始累积
        self.installation_data.z_fine_adjustment = 0.0
        
        self._initialize_fine_tuning()
    
    def _execute_post_alignment_delay(self):
        """执行对准后延时"""
        self.get_logger().info(f"步骤3.2: 对准后延时 {self.config.POST_ALIGNMENT_DELAY_TIME} 秒")
        
        # 设置延时定时器
        self._start_step_timer(self.config.POST_ALIGNMENT_DELAY_TIME, InstallationState.POST_ALIGNMENT_ADJUSTMENT)
    
    def _execute_post_alignment_adjustment(self):
        """执行对准后调整"""
        self.get_logger().info("步骤3.3: 检测到对准，向上调整3.5毫米")
        
        # 向上调整3.5mm
        self.installation_data.z_post_torque_adjustment += self.config.Z_POST_ALIGNMENT_ADJUSTMENT
        
        # 发布新的位置
        self._publish_move_command(is_approach=True)
        
        # 等待调整完成并延时
        self._start_step_timer(self.config.AXIAL_ADJUSTMENT_DELAY_TIME, InstallationState.EXECUTING_INSTALLATION)
    
    def _execute_installation(self):
        """执行安装操作"""
        self.get_logger().info("步骤4: 旋转电机进行安装")
        
        if self.installation_data.motor_status:
            target_angle = self.installation_data.motor_status.position + self.config.INSTALLATION_ROTATION
            self._rotate_motor(target_angle)
        else:
            self.get_logger().warn("没有电机状态，使用默认角度旋转")
            self._rotate_motor(1.0728)
        
        self._start_step_timer(10.0, InstallationState.MOVING_DOWN)
    
    def _execute_move_down(self):
        """执行下降移动"""
        self.get_logger().info("步骤5: 执行下降移动")
        
        self.installation_data.is_adjusting = False
        self._set_arm_velocity(self.config.MEDIUM_SPEED)
        self._publish_move_command(is_down_move=True)
        
        self._start_step_timer(15.0, InstallationState.RETRACTING)
    
    def _execute_retract(self):
        """执行回退移动"""
        self.get_logger().info("步骤6: 执行回退移动")
        
        # 停止所有正在进行的调整
        self.installation_data.is_adjusting = False
        
        # 停止所有定时器
        self._stop_all_timers()
        
        # 设置高速移动
        self._set_arm_velocity(self.config.HIGH_SPEED)
        
        # 发布回退命令
        self._publish_move_command(is_retract=True)
        
        # 添加日志确认
        self.get_logger().info("已发布回退命令，等待移动完成")
        
        # 设置完成超时定时器
        self._start_step_timer(10.0, InstallationState.COMPLETED)
        
    def _execute_completion(self):
        """执行完成操作"""
        self.get_logger().info("步骤7: 安装完成")
        
        self._send_completion_control_msg()
        self._rotate_motor(0.0)  # 复位电机
        self._reset_installation_state()
        self._transition_to_state(InstallationState.IDLE)
    
    # ============================================
    # 力矩监控和调整
    # ============================================
    
    def _check_torque_condition(self) -> bool:
        """检查力矩条件是否满足"""
        if self.torque_data.j2_torque is None or self.torque_data.j3_torque is None:
            return False
        
        if self.torque_data.last_j2_torque is None or self.torque_data.last_j3_torque is None:
            self.torque_data.last_j2_torque = self.torque_data.j2_torque
            self.torque_data.last_j3_torque = self.torque_data.j3_torque
            return False
        
        j2_diff = abs(self.torque_data.j2_torque - self.torque_data.last_j2_torque)
        j3_diff = abs(self.torque_data.j3_torque - self.torque_data.last_j3_torque)
        
        j2_condition = (self.config.J2_MIN_TORQUE_DIFF <= j2_diff <= self.config.J2_MAX_TORQUE_DIFF)
        j3_condition = (self.config.J3_MIN_TORQUE_DIFF <= j3_diff <= self.config.J3_MAX_TORQUE_DIFF)
        
        if j2_condition and j3_condition:
            self.get_logger().info(f"力矩条件满足: J2={j2_diff:.1f}, J3={j3_diff:.1f}")
            return True
        
        return False
    
    def _torque_monitor_callback(self):
        """力矩监控定时器回调"""
        if not self.installation_data.is_adjusting:
            return
        
        if self._check_torque_condition():
            self.installation_data.torque_condition_met = True
            self._stop_torque_monitoring()
            self._stop_step_timer()
            self._set_arm_velocity(self.config.VERY_LOW_SPEED)
            
            # 力矩条件满足后，不再直接提升Z轴
            self.installation_data.z_post_torque_adjustment = 0.0
            
            # 立即开始电机精调
            self._start_step_timer(0.5, InstallationState.MOTOR_FINE_TUNING)
        else:
            # 调整Z轴（累积调整）
            self.installation_data.z_adjustment += self.config.Z_ADJUST_INCREMENT
            self.get_logger().info(f"调整Z轴: +{self.installation_data.z_adjustment:.1f}mm")
            self._publish_move_command(is_approach=True)
            
            # 更新力矩记录
            self.torque_data.last_j2_torque = self.torque_data.j2_torque
            self.torque_data.last_j3_torque = self.torque_data.j3_torque
    
    def _check_sudden_torque_drop(self) -> bool:
        """检查力矩差值是否突然变小"""
        if len(self.torque_data.torque_history) < 3:
            return False
        
        # 计算最近几次的力矩差值
        recent_avg = np.mean(self.torque_data.torque_history[-3:])
        
        # 计算之前力矩差值的平均值
        if len(self.torque_data.torque_history) >= 6:
            previous_avg = np.mean(self.torque_data.torque_history[-6:-3])
        elif len(self.torque_data.torque_history) >= 4:
            previous_avg = np.mean(self.torque_data.torque_history[-4:-3])
        else:
            previous_avg = recent_avg
        
        # 检查是否突然变小
        drop_amount = previous_avg - recent_avg
        is_sudden_drop = drop_amount > self.config.TORQUE_DIFF_THRESHOLD
        
        if is_sudden_drop:
            self.get_logger().info(f"检测到力矩差值突然变小: 从{previous_avg:.2f}降到{recent_avg:.2f}, 变化量{drop_amount:.2f}")
            return True
        
        return False
    
    # ============================================
    # 精调控制 - 修改版：检测到对准后立即停止电机
    # ============================================
    
    def _fine_tuning_monitor(self):
        """电机精调过程中的力矩监控"""
        if (not self.installation_data.is_adjusting or 
            self.fine_tuning_data.alignment_detected or 
            self.fine_tuning_data.completed):
            return
        
        # 保存当前力矩差值
        if self.torque_data.j2_torque is not None and self.torque_data.j3_torque is not None:
            torque_diff = abs(self.torque_data.j2_torque - self.torque_data.j3_torque)
            self.torque_data.torque_history.append(torque_diff)
            if len(self.torque_data.torque_history) > 10:
                self.torque_data.torque_history.pop(0)
        
        # 检查力矩差值是否突然变小
        if self._check_sudden_torque_drop():
            self.fine_tuning_data.torque_stable_count += 1
            
            if self.fine_tuning_data.torque_stable_count >= self.config.REQUIRED_STABLE_COUNT:
                self.fine_tuning_data.alignment_detected = True
                self.fine_tuning_data.stop_flag = True
                self.fine_tuning_data.waiting_for_motor = False
                self.fine_tuning_data.alignment_position = self.fine_tuning_data.current_position

               
                self.get_logger().info(f"检测到对准！力矩差值稳定变小{self.fine_tuning_data.torque_stable_count}次")
                self.get_logger().info(f"对准位置: {self.fine_tuning_data.alignment_position:.3f}弧度 ({math.degrees(self.fine_tuning_data.alignment_position):.1f}°)")
                
                # 停止所有定时器
                self._stop_fine_tuning_timeout()
                self._stop_fine_tuning_timers()
                
                # 记录摆动历史汇总
                self._log_swing_summary()
                
                # 停止电机在当前位置，不返回中心位置
                self.get_logger().info("检测到对准，立即停止电机在当前位置")
                self._rotate_motor(self.fine_tuning_data.alignment_position)
                
                # 等待电机稳定
                self.get_logger().info(f"等待电机稳定 {self.config.MOTOR_SETTLE_TIME} 秒")
                self._start_step_timer(self.config.MOTOR_SETTLE_TIME, InstallationState.POST_ALIGNMENT_DELAY)
        else:
            self.fine_tuning_data.torque_stable_count = 0
    
    def _initialize_fine_tuning(self):
        """初始化精调过程"""
        self.fine_tuning_data = FineTuningData()
        
        # 重置力矩数据历史
        self.torque_data.reset()
        
        # 启动力矩监控定时器（快速检测）
        if self.fine_tuning_monitor_timer is None:
            self.fine_tuning_monitor_timer = self.create_timer(
                0.1, 
                self._fine_tuning_monitor
            )
            self.get_logger().info("开始电机精调力矩监控，每0.1秒检查一次")
        
        # 设置精调超时定时器
        self._start_fine_tuning_timeout()
        
        # 开始第一次摆动
        self._execute_fine_tuning_swing()
    
    def _start_fine_tuning_timeout(self):
        """启动精调超时定时器"""
        self._stop_fine_tuning_timeout()
        self.fine_tuning_timeout_timer = self.create_timer(
            self.config.FINE_TUNING_TIMEOUT,
            self._handle_fine_tuning_timeout
        )
    
    def _handle_fine_tuning_timeout(self):
        """处理精调超时"""
        self.get_logger().warn("电机精调试探超时，强制结束精调")
        self.fine_tuning_data.completed = True
        self.fine_tuning_data.stop_flag = True
        self.fine_tuning_data.waiting_for_motor = False
        
        # 停止所有精调定时器
        self._stop_fine_tuning_timers()
        
        # 返回电机到中心位置（超时时返回）
        self._return_motor_to_center_on_timeout()
    
    def _stop_fine_tuning_timeout(self):
        """停止精调超时定时器"""
        if self.fine_tuning_timeout_timer:
            self.fine_tuning_timeout_timer.cancel()
            self.fine_tuning_timeout_timer = None
    
    def _execute_fine_tuning_swing(self):
        """执行单次精调摆动"""
        # 检查是否应该停止精调
        if (self.fine_tuning_data.stop_flag or 
            self.fine_tuning_data.alignment_detected or 
            self.fine_tuning_data.completed):
            return
        
        # 检查是否达到最大精调次数
        if self.fine_tuning_data.step_count >= self.config.MAX_FINE_TUNING_STEPS:
            self.get_logger().warn("达到最大精调次数，未检测到对准")
            self.fine_tuning_data.completed = True
            self.fine_tuning_data.alignment_detected = False
            
            # 停止所有定时器
            self._stop_fine_tuning_timers()
            
            # 返回电机到中心位置
            self._return_motor_to_center_on_timeout()
            return
        
        # 增加总步数
        self.fine_tuning_data.step_count += 1
        
        # 计算摆动角度：固定角度 * 方向
        swing_angle = self.config.FIXED_SWING_ANGLE * self.fine_tuning_data.direction
        
        # 基于中心位置计算绝对目标角度
        # 对于序列中的每次摆动，从中心位置计算偏移
        sequence_offset = swing_angle * (self.fine_tuning_data.sequence_count + 1)
        target_angle = self.fine_tuning_data.motor_center_position + sequence_offset
        
        # 记录摆动信息
        direction_str = "正转" if self.fine_tuning_data.direction == 1 else "反转"
        deg_angle = math.degrees(self.config.FIXED_SWING_ANGLE)
        
        self.get_logger().info(f"精调步骤 {self.fine_tuning_data.step_count}: "
                            f"序列#{self.fine_tuning_data.sequence_number} "
                            f"({direction_str} {self.fine_tuning_data.sequence_count+1}/{self.fine_tuning_data.sequence_length}), "
                            f"方向={direction_str}, "
                            f"摆动角度={deg_angle:.1f}°, "
                            f"中心位置={math.degrees(self.fine_tuning_data.motor_center_position):.1f}°, "
                            f"目标位置={math.degrees(target_angle):.1f}°")
        
        # 增加当前序列计数
        self.fine_tuning_data.sequence_count += 1
        
        # 旋转电机并等待到位
        self._rotate_motor_and_wait(target_angle)
    
    def _rotate_motor_and_wait(self, target_angle: float):
        """旋转电机并等待到位"""
        try:
            # 发布电机命令
            cmd = MotorCommand()
            cmd.command_type = 6
            cmd.target_value = target_angle
            cmd.kp = self.config.MOTOR_KP
            cmd.kd = self.config.MOTOR_KD
            cmd.tau = self.config.MOTOR_TAU
            self.motor_cmd_pub.publish(cmd)
            
            self.get_logger().info(f"旋转电机到: {target_angle:.3f}弧度 ({math.degrees(target_angle):.1f}°)")
            
            # 设置等待标志和目标位置
            self.fine_tuning_data.waiting_for_motor = True
            self.fine_tuning_data.target_position = target_angle
            
            # 设置超时定时器
            self._start_motor_wait_timer(target_angle)
            
        except Exception as e:
            self._log_error("电机旋转失败", e)
            # 如果旋转失败，重置等待标志
            self.fine_tuning_data.waiting_for_motor = False
    
    def _start_motor_wait_timer(self, target_angle: float):
        """启动电机等待超时定时器"""
        if self.motor_wait_timer:
            self.motor_wait_timer.cancel()
        
        self.motor_wait_timer = self.create_timer(
            self.config.MOTOR_WAIT_TIMEOUT,
            lambda: self._handle_motor_wait_timeout(target_angle)
        )
    
    def _handle_motor_wait_timeout(self, target_angle: float):
        """处理电机等待超时"""
        if self.fine_tuning_data.waiting_for_motor:
            self.get_logger().warn(f"电机旋转超时: 目标角度={math.degrees(target_angle):.1f}°, 当前角度={math.degrees(self.fine_tuning_data.current_position):.1f}°")
            
            # 重置等待标志
            self.fine_tuning_data.waiting_for_motor = False
            
            # 停止定时器
            if self.motor_wait_timer:
                self.motor_wait_timer.cancel()
                self.motor_wait_timer = None
            
            # 强制继续精调
            self._continue_fine_tuning_after_motor_move()
    
    def _continue_fine_tuning_after_motor_move(self):
        """电机到位后继续精调流程"""
        # 检查是否需要切换序列
        if self.fine_tuning_data.is_sequence_complete():
            # 准备下一个序列
            if not self.fine_tuning_data.prepare_next_sequence():
                self.get_logger().warn("达到最大序列次数，未检测到对准")
                self.fine_tuning_data.completed = True
                self.fine_tuning_data.alignment_detected = False
                
                # 停止所有定时器
                self._stop_fine_tuning_timers()
                
                # 返回电机到中心位置
                self._return_motor_to_center_on_timeout()
                return
            
            # 在序列切换时提升Z轴
            self._adjust_z_on_sequence_switch()
        
        # 等待一段时间再进行下一次摆动
        self.get_logger().info(f"等待 {self.config.FINE_TUNING_DELAY} 秒进行下一次摆动")
        if self.step_timer:
            self.step_timer.cancel()
        
        self.step_timer = self.create_timer(
            self.config.FINE_TUNING_DELAY,
            lambda: self._execute_fine_tuning_swing()
        )
    
    def _adjust_z_on_sequence_switch(self):
        """在序列切换时调整Z轴"""
        self.installation_data.z_fine_adjustment += self.config.Z_FINE_TUNING_INCREMENT
        self.fine_tuning_data.z_adjustment_total += self.config.Z_FINE_TUNING_INCREMENT
        self.get_logger().info(f"序列切换，提升Z轴: 累计提升{self.fine_tuning_data.z_adjustment_total:.1f}mm")
        self._publish_move_command(is_approach=True)
    
    def _return_motor_to_center_on_timeout(self):
        """超时时返回电机到中心位置"""
        self.get_logger().info("超时未检测到对准，返回电机到中心位置")
        
        # 设置停止标志
        self.fine_tuning_data.stop_flag = True
        self.fine_tuning_data.waiting_for_motor = False
        
        # 记录摆动历史汇总
        self._log_swing_summary()
        
        # 停止所有精调定时器
        self._stop_fine_tuning_timers()
        
        # 返回电机到中心位置
        center_position = self.fine_tuning_data.motor_center_position
        self._rotate_motor(center_position)
        
        # 等待电机返回中心位置
        self.get_logger().info(f"等待电机返回中心位置 {self.config.MOTOR_SETTLE_TIME} 秒")
        self._start_step_timer(self.config.MOTOR_SETTLE_TIME, InstallationState.EXECUTING_INSTALLATION)
    
    def _log_swing_summary(self):
        """记录摆动历史汇总"""
        self.get_logger().info("=== 摆动历史汇总 ===")
        self.get_logger().info(f"总摆动次数: {self.fine_tuning_data.step_count}")
        self.get_logger().info(f"完成的序列数: {self.fine_tuning_data.sequence_number - 1}")
        self.get_logger().info(f"总Z轴提升: {self.fine_tuning_data.z_adjustment_total:.1f}mm")
        self.get_logger().info(f"是否检测到对准: {self.fine_tuning_data.alignment_detected}")
        if self.fine_tuning_data.alignment_detected:
            self.get_logger().info(f"对准位置: {math.degrees(self.fine_tuning_data.alignment_position):.1f}°")
        
        # 显示摆动模式
        self.get_logger().info(f"摆动模式: 固定角度{math.degrees(self.config.FIXED_SWING_ANGLE):.1f}°, "
                             f"正转1次, 反转2次, 正转3次... 最大{self.config.MAX_SEQUENCE_COUNT}次")
        self.get_logger().info(f"Z轴提升策略: 每次正反转切换时提升{self.config.Z_FINE_TUNING_INCREMENT}mm")
    
    # ============================================
    # 运动控制
    # ============================================
    
    def _publish_move_command(self, **kwargs):
        """发布移动命令"""
        if self.installation_data.target_pose is None:
            self.get_logger().warn("无法发布移动命令：没有目标位置信息")
            return
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        
        # 计算Z轴位置
        z_adjustment_total = (
            self.installation_data.z_adjustment +
            self.installation_data.z_fine_adjustment +
            self.installation_data.z_post_torque_adjustment
        )
        
        # 根据不同的移动类型设置位置
        if kwargs.get('is_retract', False):
            # 回退到安全位置
            pose_msg.pose.position.x = self.config.SAFE_POSITION[0]
            pose_msg.pose.position.y = self.config.SAFE_POSITION[1]
            pose_msg.pose.position.z = self.config.SAFE_POSITION[2]
            quat = quaternion_from_euler(*self.config.SAFE_ORIENTATION_EULER)
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            
            self.get_logger().info(f"回退到安全位置: ({pose_msg.pose.position.x}, "
                                f"{pose_msg.pose.position.y}, {pose_msg.pose.position.z})")
            
        elif kwargs.get('is_down_move', False):
            # 下降移动
            pose_msg.pose.position.x = self.installation_data.target_pose.pose.position.x
            pose_msg.pose.position.y = self.installation_data.target_pose.pose.position.y
            pose_msg.pose.position.z = (
                self.installation_data.target_pose.pose.position.z - 
                self.config.DOWN_MOVE_OFFSET
            )
            pose_msg.pose.orientation = self.installation_data.target_pose.pose.orientation
            
        elif kwargs.get('is_safe_height', False):
            # 安全高度移动
            pose_msg.pose.position.x = self.installation_data.target_pose.pose.position.x
            pose_msg.pose.position.y = self.installation_data.target_pose.pose.position.y
            pose_msg.pose.position.z = (
                self.installation_data.target_pose.pose.position.z - 
                self.config.SAFE_HEIGHT_OFFSET
            )
            pose_msg.pose.orientation = self.installation_data.target_pose.pose.orientation
            
        elif kwargs.get('is_approach', False):
            # 接近目标
            pose_msg.pose.position.x = self.installation_data.target_pose.pose.position.x
            pose_msg.pose.position.y = self.installation_data.target_pose.pose.position.y
            pose_msg.pose.position.z = (
                self.installation_data.target_pose.pose.position.z - 
                self.config.APPROACH_HEIGHT_OFFSET + 
                z_adjustment_total
            )
            pose_msg.pose.orientation = self.installation_data.target_pose.pose.orientation
        
        # 发布命令
        self.target_pose_pub.publish(pose_msg)
        
        # 发布控制消息
        control_msg = ControlMsg()
        control_msg.frequency = self.installation_data.install_frequency + 1
        control_msg.switch_control = True
        self.control_pub.publish(control_msg)
        
        # 记录日志
        log_msg = ""
        if kwargs.get('is_retract', False):
            log_msg = f"回退到安全位置"
        elif kwargs.get('is_down_move', False):
            log_msg = f"下降移动"
        elif kwargs.get('is_safe_height', False):
            log_msg = f"移动到安全高度"
        elif kwargs.get('is_approach', False):
            if self.installation_data.torque_condition_met:
                log_msg = f"力矩调整完成: Z轴总调整={z_adjustment_total:.1f}mm"
            elif self.installation_data.is_adjusting:
                log_msg = f"电机精调中: Z轴总调整={z_adjustment_total:.1f}mm"
            else:
                log_msg = f"接近目标: Z轴总调整={z_adjustment_total:.1f}mm"
    
    def _rotate_motor(self, target_angle: float):
        """旋转电机到指定角度"""
        try:
            cmd = MotorCommand()
            cmd.command_type = 6
            cmd.target_value = target_angle
            cmd.kp = self.config.MOTOR_KP
            cmd.kd = self.config.MOTOR_KD
            cmd.tau = self.config.MOTOR_TAU
            self.motor_cmd_pub.publish(cmd)
            self.get_logger().info(f"旋转电机到: {target_angle:.3f}弧度 ({math.degrees(target_angle):.1f}°)")
        except Exception as e:
            self._log_error("电机旋转失败", e)
    
    # ============================================
    # 定时器管理
    # ============================================
    
    def _start_step_timer(self, duration: float, next_state: InstallationState):
        """启动步骤定时器"""
        self._stop_step_timer()
        self.step_timer = self.create_timer(duration, 
                                          lambda: self._handle_step_timeout(next_state))
    
    def _handle_step_timeout(self, next_state: InstallationState):
        """处理步骤超时"""
        self._stop_step_timer()
        self._transition_to_state(next_state)
    
    def _start_torque_monitoring(self):
        """启动力矩监控"""
        self._stop_torque_monitoring()
        self.torque_check_timer = self.create_timer(
            self.config.TORQUE_CHECK_INTERVAL,
            self._torque_monitor_callback
        )
    
    def _stop_all_timers(self):
        """停止所有定时器"""
        self._stop_step_timer()
        self._stop_torque_monitoring()
        self._stop_fine_tuning_timers()
    
    def _stop_step_timer(self):
        if self.step_timer:
            self.step_timer.cancel()
            self.step_timer = None
    
    def _stop_torque_monitoring(self):
        if self.torque_check_timer:
            self.torque_check_timer.cancel()
            self.torque_check_timer = None
    
    def _stop_fine_tuning_timers(self):
        """停止所有精调相关的定时器"""
        if self.torque_check_timer:
            self.torque_check_timer.cancel()
            self.torque_check_timer = None
        
        if self.fine_tuning_monitor_timer:
            self.fine_tuning_monitor_timer.cancel()
            self.fine_tuning_monitor_timer = None
        
        if self.motor_wait_timer:
            self.motor_wait_timer.cancel()
            self.motor_wait_timer = None
        
        self._stop_fine_tuning_timeout()
    
    # ============================================
    # 辅助函数
    # ============================================
    
    def _set_arm_velocity(self, velocity: float):
        """设置机械臂速度"""
        try:
            speed_msg = Float64()
            speed_msg.data = velocity
            self.arm_velocity_pub.publish(speed_msg)
            self.get_logger().info(f"发布速度: {velocity}")
        except Exception as e:
            self._log_error("发布速度失败", e)
    
    def _send_completion_control_msg(self):
        """发送完成控制消息"""
        control_msg = ControlMsg()
        control_msg.switch_control = True
        control_msg.frequency = self.installation_data.install_frequency + 1
        self.control_pub.publish(control_msg)
        self.get_logger().info(f"安装完成. 频次更新为: {control_msg.frequency}")
    
    def _publish_process_message(self, message: str):
        """发布处理进度消息"""
        msg = String()
        msg.data = message
        self.process_pub.publish(msg)
    
    def _log_error(self, context: str, error: Exception, traceback: bool = False):
        """统一错误日志记录"""
        self.get_logger().error(f"{context}: {str(error)}")
        
        if traceback:
            import traceback
            self.get_logger().error(f"详细错误信息:\n{traceback.format_exc()}")
        
        # 发布错误消息
        error_msg = String()
        error_msg.data = f"{context}: {str(error)}"
        self.error_pub.publish(error_msg)
    
    def _log_pose_info(self, message: str, position):
        """记录位置信息"""
        self.get_logger().info(
            f"{message}: x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}"
        )


# ============================================
# 主函数
# ============================================
def main(args=None):
    rclpy.init(args=args)
    try:
        node = InstallationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("安装节点已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()