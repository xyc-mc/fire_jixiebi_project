#!/usr/bin/env python3
"""
测试手眼标定转换
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import numpy as np
import time

class TestCameraPointPublisher(Node):
    """测试用节点：发布相机坐标"""
    
    def __init__(self):
        super().__init__('test_camera_publisher')
        
        # 发布相机坐标
        self.publisher = self.create_publisher(
            PointStamped,
            '/camera/point',
            10
        )
        
        # 订阅TCP坐标
        self.subscription = self.create_subscription(
            PointStamped,
            '/tcp/point',
            self.tcp_point_callback,
            10
        )
        
        self.test_points = [
            [0.5, 0.3, 1.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [0.0, 1.0, 1.0],
        ]
        
        self.current_test = 0
        
        # 定时器：每秒发布一个测试点
        self.timer = self.create_timer(2.0, self.publish_test_point)
        
    def publish_test_point(self):
        """发布测试点"""
        if self.current_test < len(self.test_points):
            point = self.test_points[self.current_test]
            
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            msg.point.x = float(point[0])
            msg.point.y = float(point[1])
            msg.point.z = float(point[2])
            
            self.publisher.publish(msg)
            self.get_logger().info(f"发布相机坐标: {point}")
            
            self.current_test += 1
        else:
            self.get_logger().info("所有测试点已发布")
            self.timer.cancel()
    
    def tcp_point_callback(self, msg):
        """接收转换后的TCP坐标"""
        self.get_logger().info(
            f"收到TCP坐标: [{msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}]"
        )

def main(args=None):
    rclpy.init(args=args)
    
    # 创建测试节点
    test_node = TestCameraPointPublisher()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info("测试节点被中断")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()