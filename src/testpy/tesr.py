#!/usr/bin/env python3
# test_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.create_subscription(
            PointStamped,
            '/camera/point',
            self.callback,
            10
        )
        print("测试节点已启动")
    
    def callback(self, msg):
        print(f"收到消息: x={msg.point.x}, y={msg.point.y}, z={msg.point.z}")

def main():
    rclpy.init()
    node = TestNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()