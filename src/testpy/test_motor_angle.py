#!/usr/bin/env python3
"""
Unitree电机控制测试脚本 - 简单版
功能：
1. 获取当前电机角度
2. 输入角度增量，计算目标位置
3. 控制电机转动到目标位置
4. 实时显示位置和力矩信息
"""

import rclpy
from rclpy.node import Node
import time
import math
from datetime import datetime

from unitreemotor_msgs.msg import MotorCommand, MotorStatus
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SimpleMotorTest(Node):
    def __init__(self):
        super().__init__('simple_motor_test')
        
        # 初始化变量
        self.current_position = 0.0
        self.current_torque = 0.0
        self.current_velocity = 0.0
        self.is_running = False
        
        # 创建QoS配置
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        # 创建发布者 - 发送控制命令
        self.command_pub = self.create_publisher(
            MotorCommand, 
            'motor/command', 
            qos_profile
        )
        
        # 创建订阅者 - 接收状态信息
        self.status_sub = self.create_subscription(
            MotorStatus,
            'motor/status',
            self.status_callback,
            qos_profile
        )
        
        # 创建定时器用于更新显示
        self.display_timer = self.create_timer(0.1, self.update_display)
        
        self.get_logger().info("简单电机测试节点启动")
        self.get_logger().info("输入 'help' 查看可用命令")
        
        # 启动命令行接口
        self.start_command_line()
    
    def status_callback(self, msg):
        """状态回调函数 - 更新当前状态"""
        self.current_position = msg.position
        self.current_velocity = msg.velocity
        self.current_torque = msg.torque
        
        # 可以在这里添加数据记录或其他处理
    
    def update_display(self):
        """更新显示 - 可以在这里添加实时图表或其他可视化"""
        pass  # 当前仅命令行显示
    
    def get_current_angle(self):
        """获取当前角度"""
        return self.current_position
    
    def move_to_angle(self, target_angle, kp=5.0, kd=1.0):
        """
        移动到指定角度
        
        Args:
            target_angle: 目标角度(弧度)
            kp: 比例增益
            kd: 微分增益
        """
        self.get_logger().info(f"准备移动到: {target_angle:.3f} rad")
        
        # 创建位置控制命令
        command = MotorCommand()
        command.command_type = 1  # POSITION_CONTROL
        command.target_value = float(target_angle)
        command.kp = float(kp)
        command.kd = float(kd)
        command.max_velocity = 5.0  # 限制最大速度
        command.max_torque = 3.0    # 限制最大力矩
        
        # 发布命令
        self.command_pub.publish(command)
        
        # 等待一小段时间让命令生效
        time.sleep(0.1)
        
        self.get_logger().info(f"位置命令已发送: {target_angle:.3f} rad")
        return True
    
    def move_by_delta(self, delta_angle, kp=5.0, kd=1.0):
        """
        移动指定角度增量
        
        Args:
            delta_angle: 角度增量(弧度)
            kp: 比例增益
            kd: 微分增益
        """
        current = self.get_current_angle()
        target = current + delta_angle
        
        self.get_logger().info(f"当前位置: {current:.3f} rad")
        self.get_logger().info(f"角度增量: {delta_angle:.3f} rad")
        self.get_logger().info(f"目标位置: {target:.3f} rad")
        
        return self.move_to_angle(target, kp, kd)
    
    def wait_for_position(self, target_angle, timeout=3.0, tolerance=0.05):
        """
        等待到达目标位置
        
        Args:
            target_angle: 目标角度
            timeout: 超时时间(秒)
            tolerance: 位置容差(弧度)
        """
        start_time = time.time()
        last_error = None
        
        self.get_logger().info(f"等待到达目标位置: {target_angle:.3f} rad")
        self.get_logger().info(f"容差: {tolerance:.3f} rad, 超时: {timeout}秒")
        
        while rclpy.ok():
            current = self.get_current_angle()
            error = abs(current - target_angle)
            
            # 显示当前位置和误差
            print(f"\r当前位置: {current:.4f} rad, 误差: {error:.4f} rad, 力矩: {self.current_torque:.3f} Nm", end="")
            
            # 检查是否到达目标
            if error < tolerance:
                print()  # 换行
                self.get_logger().info(f"到达目标位置! 最终误差: {error:.4f} rad")
                return True
            
            # 检查超时
            if time.time() - start_time > timeout:
                print()  # 换行
                self.get_logger().warning(f"超时! 当前位置: {current:.4f} rad, 目标: {target_angle:.4f} rad")
                return False
            
            # 检查误差变化（如果误差不再减小，可能已经稳定）
            if last_error is not None and abs(error - last_error) < 0.001:
                if time.time() - start_time > 1.0:  # 至少等待1秒
                    print()  # 换行
                    self.get_logger().info(f"位置已稳定! 当前位置: {current:.4f} rad, 最终误差: {error:.4f} rad")
                    return True
            
            last_error = error
            time.sleep(0.05)  # 50ms更新一次
        
        return False
    
    def print_current_status(self):
        """打印当前状态"""
        print(f"{'='*50}")
        print(f"时间: {datetime.now().strftime('%H:%M:%S')}")
        print(f"当前位置: {self.current_position:.4f} rad")
        print(f"当前速度: {self.current_velocity:.4f} rad/s")
        print(f"当前力矩: {self.current_torque:.4f} Nm")
        print(f"{'='*50}")
    
    def start_command_line(self):
        """启动命令行交互"""
        # 在新线程中运行命令行接口
        import threading
        cli_thread = threading.Thread(target=self.command_line_interface, daemon=True)
        cli_thread.start()
    
    def command_line_interface(self):
        """命令行交互接口"""
        time.sleep(1)  # 等待ROS初始化
        
        print("\n" + "="*60)
        print("Unitree电机控制测试工具")
        print("="*60)
        
        while rclpy.ok():
            try:
                print("\n当前选项:")
                print("1. 获取当前位置")
                print("2. 移动到绝对角度")
                print("3. 移动角度增量")
                print("4. 显示状态信息")
                print("5. 停止电机")
                print("6. 退出")
                
                choice = input("\n请选择操作 (1-6): ").strip()
                
                if choice == '1':
                    # 获取当前位置
                    print(f"\n当前位置: {self.get_current_angle():.4f} rad")
                
                elif choice == '2':
                    # 移动到绝对角度
                    try:
                        angle_str = input("请输入目标角度(弧度): ").strip()
                        angle = float(angle_str)
                        
                        # 获取可选参数
                        kp_str = input(f"请输入kp值(默认5.0): ").strip()
                        kd_str = input(f"请输入kd值(默认1.0): ").strip()
                        
                        kp = float(kp_str) if kp_str else 5.0
                        kd = float(kd_str) if kd_str else 1.0
                        
                        self.move_to_angle(angle, kp, kd)
                        self.wait_for_position(angle)
                        
                    except ValueError:
                        print("错误: 请输入有效的数字")
                
                elif choice == '3':
                    # 移动角度增量
                    try:
                        delta_str = input("请输入角度增量(弧度): ").strip()
                        delta = float(delta_str)
                        
                        # 获取可选参数
                        kp_str = input(f"请输入kp值(默认5.0): ").strip()
                        kd_str = input(f"请输入kd值(默认1.0): ").strip()
                        
                        kp = float(kp_str) if kp_str else 5.0
                        kd = float(kd_str) if kd_str else 1.0
                        
                        current = self.get_current_angle()
                        target = current + delta
                        
                        self.move_by_delta(delta, kp, kd)
                        self.wait_for_position(target)
                        
                    except ValueError:
                        print("错误: 请输入有效的数字")
                
                elif choice == '4':
                    # 显示状态信息
                    self.print_current_status()
                
                elif choice == '5':
                    # 停止电机
                    command = MotorCommand()
                    command.command_type = 0  # STOP
                    command.target_value = 0.0
                    command.kp = 0.0
                    command.kd = 0.0
                    
                    self.command_pub.publish(command)
                    print("已发送停止命令")
                
                elif choice == '6':
                    # 退出
                    print("正在退出...")
                    self.destroy_node()
                    rclpy.shutdown()
                    break
                
                else:
                    print("错误: 无效的选择")
            
            except KeyboardInterrupt:
                print("\n正在退出...")
                self.destroy_node()
                rclpy.shutdown()
                break
            except Exception as e:
                print(f"发生错误: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimpleMotorTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        print("测试结束")

if __name__ == '__main__':
    main()