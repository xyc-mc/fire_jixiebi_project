import rclpy
from rclpy.node import Node
from base_msgs.msg import ControlMsg
import os
from sensor_msgs.msg import PointCloud2, PointField, Image


class Client_Pub(Node):
    def __init__(self):
        super().__init__('Client_Pub_node')
        self.control_pub = self.create_publisher(ControlMsg, '/sport_control', 10)
        self.get_logger().info('等待订阅者...')
        while self.control_pub.get_subscription_count() == 0:
            # 等待0.1秒
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('订阅者已连接，开始发布')
        self.control()

    def control(self):
        control_msg = ControlMsg()
        control_msg.center[0] = 200.0
        control_msg.center[1] = 80.0
        control_msg.center[2] = 500.0
        control_msg.switch_control = True
        control_msg.frequency = 1
        self.control_pub.publish(control_msg)

    

    
def main():
    rclpy.init()
    client = Client_Pub()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()