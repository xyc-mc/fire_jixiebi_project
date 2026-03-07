#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from unitreemotor_msgs.srv import MotorControl
import time
import math

class MotorModeTest(Node):
    def __init__(self):
        super().__init__('motor_mode_test')
        
        # 创建服务客户端
        self.client = self.create_client(MotorControl, 'motor/control')
        
        # 等待服务
        self.client.wait_for_service()
        self.get_logger().info("服务已连接")
        
    def set_position_mode(self, angle=math.pi/2, kp=0.01, kd=0.01, duration=5.0):
        """设置位置模式"""
        self.get_logger().info(f"设置位置模式: 角度={angle:.3f} rad, kp={kp:.3f}, kd={kd:.3f}")
        request = MotorControl.Request()
        request.command = 1  # 位置模式
        request.value = angle
        request.duration = duration
        request.kp = kp
        request.kd = kd
        request.max_velocity = 5.0
        request.max_torque = 2.0
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"响应: {response.message}")
            return response.success
        return False
    
    def set_velocity_mode(self, velocity=3.14, kd=0.01, duration=3.0):
        """设置速度模式"""
        self.get_logger().info(f"设置速度模式: 速度={velocity:.3f} rad/s, kd={kd:.3f}")
        request = MotorControl.Request()
        request.command = 2  # 速度模式
        request.value = velocity
        request.duration = duration
        request.kd = kd
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"响应: {response.message}")
            return response.success
        return False
    
    def set_damping_mode(self, kd=0.01, duration=3.0):
        """设置阻尼模式"""
        self.get_logger().info(f"设置阻尼模式: kd={kd:.3f}")
        request = MotorControl.Request()
        request.command = 3  # 阻尼模式
        request.duration = duration
        request.kd = kd
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"响应: {response.message}")
            return response.success
        return False
    
    def set_torque_mode(self, torque=0.1, duration=3.0):
        """设置力矩模式"""
        self.get_logger().info(f"设置力矩模式: 力矩={torque:.3f} Nm")
        request = MotorControl.Request()
        request.command = 4  # 力矩模式
        request.value = torque
        request.duration = duration
        request.max_torque = 1.0
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"响应: {response.message}")
            return response.success
        return False
    
    def set_zero_torque_mode(self, duration=3.0):
        """设置零力矩模式"""
        self.get_logger().info("设置零力矩模式")
        request = MotorControl.Request()
        request.command = 5  # 零力矩模式
        request.duration = duration
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"响应: {response.message}")
            return response.success
        return False

def main():
    rclpy.init()
    node = MotorModeTest()
    
    try:
        # 测试位置模式
        print("\n=== 测试位置模式 ===")
        if node.set_position_mode(angle=math.pi/2, kp=0.01, kd=0.01, duration=3.0):
            time.sleep(4)
        
        # 测试速度模式
        print("\n=== 测试速度模式 ===")
        if node.set_velocity_mode(velocity=2.0, kd=0.01, duration=3.0):
            time.sleep(4)
        
        # 测试阻尼模式
        print("\n=== 测试阻尼模式 ===")
        if node.set_damping_mode(kd=0.02, duration=2.0):
            time.sleep(3)
        
        # 测试力矩模式
        print("\n=== 测试力矩模式 ===")
        if node.set_torque_mode(torque=0.06, duration=3.0):
            time.sleep(4)
        
        # 测试零力矩模式
        print("\n=== 测试零力矩模式 ===")
        if node.set_zero_torque_mode(duration=2.0):
            time.sleep(3)
        
        # 回到零位
        print("\n=== 回到零位 ===")
        if node.set_position_mode(angle=0.0, kp=0.01, kd=0.01, duration=3.0):
            time.sleep(4)
        
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()