#!/usr/bin/env python3
"""
超简化的正运动学测试程序
只关注数组输入和数组输出
"""
import rclpy
from rclpy.node import Node
from elite_msgs.srv import ForwardKinematic
import transforms3d as tfs


class SimpleFKTester(Node):
    def __init__(self):
        super().__init__('simple_fk_tester')
        self.fk_client = self.create_client(ForwardKinematic, 'forward_kinematics')
        self.wait_for_service()
    
    def wait_for_service(self):
        while not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('等待正运动学服务...')
    
    def calculate_fk(self, joints_array):
        """
        输入: joints_array = [j1, j2, j3, j4, j5, j6] (弧度)
        输出: [x, y, z, rx, ry, rz] (米, 弧度) 或 None
        """
        # 创建请求
        request = ForwardKinematic.Request()
        
        # 设置关节角度
        request.joint = joints_array
        
        # 调用服务
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            pose = response.pose
            
            # 四元数转欧拉角
            quat = [
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            ]
            euler = tfs.euler.quat2euler(quat)
            
            # 返回笛卡尔坐标
            return [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                euler[0],
                euler[1],
                euler[2]
            ]
        return None


# 直接使用示例
def test_example():
    rclpy.init()
    
    # 创建测试器
    tester = SimpleFKTester()
    
    # 测试数据（基于逆解程序的反向测试）
    # 假设这是已知的关节角度
    joints = [12.638, -96.195, 117.949, 254.570, 269.551,-334.491]  # [j1, j2, j3, j4, j5, j6]
    
    # 注意：需要转换为弧度（根据第二个程序的服务端逻辑）
    import math
    joints_rad = [math.radians(j) for j in joints]
    
    # 计算正解
    result = tester.calculate_fk(joints_rad)
    
    # 输出结果
    print(f"输入关节角度 (度): {joints}")
    print(f"输入关节角度 (弧度): {joints_rad}")
    if result:
        print(f"输出位姿:")
        print(f"  位置 X: {result[0]:.6f} m")
        print(f"  位置 Y: {result[1]:.6f} m")
        print(f"  位置 Z: {result[2]:.6f} m")
        print(f"  姿态 Rx: {result[3]:.6f} rad ({math.degrees(result[3]):.2f}°)")
        print(f"  姿态 Ry: {result[4]:.6f} rad ({math.degrees(result[4]):.2f}°)")
        print(f"  姿态 Rz: {result[5]:.6f} rad ({math.degrees(result[5]):.2f}°)")
    else:
        print("正运动学计算失败")
    
    # 清理
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    test_example()