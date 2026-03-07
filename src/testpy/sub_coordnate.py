
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from elite_msgs.msg import RobotState
import math
import threading
import time

class SimpleTCPSubscriber(Node):
    def __init__(self):
        super().__init__('simple_tcp_subscriber')
        
        # 存储接收到的TCP位姿
        self.tcp_pose = None
        self.pose_received = False
        
        # 创建订阅
        self.subscription = self.create_subscription(
            RobotState,
            '/robot_state',
            self.callback,
            10
        )
    
    def callback(self, msg):
        """处理消息的回调函数"""
        # 如果已经接收到数据，直接返回（可选）   
        if not self.pose_received:
            try:
                if len(msg.machine_pose) >= 6:
                    x, y, z, rx, ry, rz = msg.machine_pose[:6]
                    0
                    # 转换为可读格式
                    rx_deg = math.degrees(rx)
                    ry_deg = math.degrees(ry)
                    rz_deg = math.degrees(rz)
                    
                    # 存储数据
                    self.tcp_pose = {
                        'position_mm': [x, y, z],
                        'orientation_rad': [rx, ry, rz],
                        'orientation_deg': [rx_deg, ry_deg, rz_deg],
                        'raw_machine_pose': msg.machine_pose[:6]
                    }
                    
                    self.pose_received = True
                    
                    # 输出到控制台
                    print(f"\n{'='*60}")
                    print(f"成功接收到TCP位姿数据:")
                    print(f"位置 (mm): X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
                    print(f"欧拉角 (度): RX={rx_deg:.2f}, RY={ry_deg:.2f}, RZ={rz_deg:.2f}")
                    print(f"欧拉角 (弧度): RX={rx:.4f}, RY={ry:.4f}, RZ={rz:.4f}")
                    print(f"{'='*60}")

            except Exception as e:
                self.get_logger().error(f"处理消息时出错: {e}")
    


def main():
    rclpy.init()
    node = SimpleTCPSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
   

if __name__ == '__main__':
    main()