#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time
import sys

class SimplePosePublisher(Node):
    def __init__(self):
        super().__init__('simple_pose_publisher')
        
        # 创建发布者
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/target_pose',
            10
        )
        
        self.get_logger().info('简单坐标发布器启动')
    
    def publish_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """发布目标位姿"""
        # 创建位姿消息
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        
        # 设置位置 (毫米)
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        
        # 设置姿态 (四元数)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        
        # 发布位姿
        self.pose_publisher.publish(pose_msg)
        
        self.get_logger().info(
            f"发布目标位姿: 位置=({x:.3f}, {y:.3f}, {z:.3f})毫米"
        )
        
        return pose_msg


def main():
    rclpy.init()
    
    # 创建节点
    publisher = SimplePosePublisher()
    
    # 等待发布者建立连接
    time.sleep(1.0)
    
    try:
        # 如果有命令行参数，使用参数作为坐标
        if len(sys.argv) >= 4:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            z = float(sys.argv[3])
        else:
            # 默认坐标 (毫米)
            x = -120.0
            y = 420.0   # Y坐标
            z = 520.0   # Z坐标
        
        # 发布位姿
        publisher.publish_pose(x, y, z)
        
        # 等待一会儿，确保消息被发送
        time.sleep(0.5)
        
        # 打印完成信息
        print("="*60)
        print(f"已发布目标位姿到 /target_pose 话题")
        print(f"坐标: X={x:.3f}米, Y={y:.3f}米, Z={z:.3f}米")
        print("="*60)
        
    except ValueError:
        print("错误：坐标参数必须是数字")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        # 清理并退出
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()