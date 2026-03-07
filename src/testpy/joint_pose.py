#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class SimpleJointSubscriber(Node):
    def __init__(self):
        super().__init__('simple_joint_subscriber')
        
        # 存储接收到的关节数据
        self.joint_pose = None
        self.pose_received = False
        
        # 创建订阅，订阅/joint_states话题
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # 订阅机械臂发布的关节状态话题
            self.callback,
            10
        )
        self.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        self.get_logger().info("等待关节状态数据...")

    def callback(self, msg):
        """处理消息的回调函数"""
        # 如果已经接收到数据，直接返回（只接收一次）
        if not self.pose_received:
            try:
                if len(msg.position) >= 6:  # 使用JointState的标准字段position
                    j1, j2, j3, j4, j5, j6 = msg.position[:6]
                    
                    # 转换为可读格式
                    j1_deg = math.degrees(j1)
                    j2_deg = math.degrees(j2)
                    j3_deg = math.degrees(j3)
                    j4_deg = math.degrees(j4)
                    j5_deg = math.degrees(j5)
                    j6_deg = math.degrees(j6)

                    # 存储数据
                    self.tcp_pose = {
                        'orientation_rad': [j1, j2, j3, j4, j5, j6],
                        'orientation_deg': [j1_deg, j2_deg, j3_deg, j4_deg, j5_deg, j6_deg],
                        'raw_joint_pose': msg.position[:6]
                    }
                    
                    self.pose_received = True
                    
                    # 输出到控制台
                    print(f"\n{'='*60}")
                    print(f"成功接收到关节角度数据:")
                    print(f"关节 (度): Rj1={j1_deg:.2f}, Rj2={j2_deg:.2f} Rj3={j3_deg:.2f}, Rj4={j4_deg:.2f}, Rj5={j5_deg:.2f}, Rj6={j6_deg:.2f}")
                    print(f"关节 (弧度): Rj1={j1:.4f}, Rj2={j2:.4f} Rj3={j3:.4f}, Rj4={j4:.4f}, Rj5={j5:.4f}, Rj6={j6:.4f}")
                    print(f"{'='*60}")
                    
                    # 打印后可以退出或继续运行（根据你的需求）
                    # 如果需要接收一次后退出，可以取消下面两行注释
                    # self.get_logger().info("已接收到一次数据，节点继续运行但不再处理新数据...")
                    
            except Exception as e:
                self.get_logger().error(f"处理消息时出错: {e}")


def main():
    rclpy.init()
    node = SimpleJointSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
   

if __name__ == '__main__':
    main()