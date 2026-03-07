#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from elite_msgs.srv import InverseKinematic
import transforms3d as tfs
import math


class SimpleIKTester(Node):
    def __init__(self):
        super().__init__('simple_ik_tester')
        self.ik_client = self.create_client(InverseKinematic, 'inverse_kinematics')
        self.wait_for_service()
    
    def wait_for_service(self):
        while not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('等待逆运动学服务...')
    
    def calculate_ik(self, target_pose_array, ref_joints_array):
        """
        输入: target_pose_array = [x, y, z, rx, ry, rz] (毫米, 角度)
              ref_joints_array = [j1, j2, j3, j4, j5, j6] (角度)
        输出: [j1, j2, j3, j4, j5, j6] (角度) 或 None
        """
        from geometry_msgs.msg import Pose
        
        # 创建请求
        request = InverseKinematic.Request()
        
        # 设置位置
        request.pose = Pose()
        request.pose.position.x = target_pose_array[0]
        request.pose.position.y = target_pose_array[1]
        request.pose.position.z = target_pose_array[2]
        
        # 欧拉角转四元数
        quat = tfs.euler.euler2quat(
            math.radians(target_pose_array[3]), 
            math.radians(target_pose_array[4]), 
            math.radians(target_pose_array[5]))
        
        request.pose.orientation.w = quat[0]
        request.pose.orientation.x = quat[1]
        request.pose.orientation.y = quat[2]
        request.pose.orientation.z = quat[3]
        
        # 设置参考关节
        request.ref_joint = ref_joints_array
        
        # 调用服务
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.result:
                return list(response.joint)
        return None


# 直接使用示例
def test_example():
    rclpy.init()
    
    # 创建测试器
    tester = SimpleIKTester()
    
    # 测试数据
    target_pose = [-150.000, 440.000, 600.000, -8.8, 3.8, -123.8]  # [x, y, z, rx, ry, rz]
    ref_joints = [90.0, -90.0, 100.0, 250.0, 271.0, -308.0]     # [j1, j2, j3, j4, j5, j6]
    
    # 计算逆解
    result = tester.calculate_ik(target_pose, ref_joints)
    
    # 输出结果
    print(f"输入目标位姿: {target_pose}")
    print(f"输入参考关节: {ref_joints}")
    print(f"输出关节角度: {result}")
    
    # 清理
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    test_example()