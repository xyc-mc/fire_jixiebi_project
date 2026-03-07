import rclpy
from rclpy.node import Node
from base_msgs.msg import ControlMsg, DataSend
from elite_msgs.msg import RobotState
import os
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class ProgramEntrance(Node):
    def __init__(self):
        super().__init__('program_entrance_node')

        self.control_pub = self.create_publisher(ControlMsg, '/camera_control', 10)
        self.start_sub = self.create_subscription(String, '/start', self.start_callback, 10)
        
        self.process_data_pub = self.create_publisher(String, '/process_data', 10)
        self.work_result_pub = self.create_publisher(String, '/work_result', 10)
        self.error_data_pub = self.create_publisher(String, '/error_data', 10)
        self.effort_pub = self.create_publisher(String, '/joint_effort', 10)
        self.torque_sub = self.create_subscription(RobotState, '/robot_state', self.torque_callback, 10)

    def start_callback(self, data):
        if data.data == 'start':
            self.reset_data()
            control_msg = ControlMsg()
            control_msg.switch_control = True
            control_msg.frequency = 1
            self.control_pub.publish(control_msg)
        else:
            self.reset_data()
            self.get_logger().error('开始指令错误，请重试！')
    
    def torque_callback(self, msg):
        a = []
        for i in range(6):
            a.append(msg.torque[i])
        joint_state = String()       
        joint_state.data = str(a)
        self.effort_pub.publish(joint_state)

    def reset_data(self):
        process_data = String()
        error_data = String()
        work_result = String()

        process_data.data = str(None)
        error_data.data = str(None)
        work_result.data = str(None)

        self.process_data_pub.publish(process_data)
        self.error_data_pub.publish(error_data)
        self.work_result_pub.publish(work_result)

def main():
    rclpy.init()
    entrance = ProgramEntrance()
    rclpy.spin(entrance)
    rclpy.shutdown()

if __name__ == '__main__':
    main()