import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64,String
from base_msgs.msg import ControlMsg  
from elite_msgs.msg import RobotState

class FirstScan(Node):
    def __init__(self):
        super().__init__('first_scan')
        
        self.first_pose_stamped = None
        self.current_robot_state = None
        self.pending_control_msg = None
      
        
        self.subscription = self.create_subscription(
            PoseStamped, '/target_pose', self.listener_callback, 10)
        self.control_subscription = self.create_subscription(
            ControlMsg, '/sport_control', self.control_callback, 10)
        self.pose_subscription = self.create_subscription(
            RobotState, '/robot_state', self.current_callback, 10)
            
        self.publisher = self.create_publisher(PoseStamped, '/move_pose', 10)
        self.control_publisher = self.create_publisher(ControlMsg, '/move_control', 10)
        self.arm_velocity_publisher = self.create_publisher(Float64, '/arm_velocity', 10)
        
        self.process_pub = self.create_publisher(String, '/process_data', 10)
        self.error_pub = self.create_publisher(String, '/error_data', 10)


    def listener_callback(self, msg):
        if self.first_pose_stamped is None:
            self.first_pose_stamped = msg
            self.get_logger().info(f"保存初始位姿")
            
            if self.pending_control_msg is not None:
                self.process_control_command(self.pending_control_msg)
                self.pending_control_msg = None
    
    def current_callback(self, msg):
        self.current_robot_state = msg
    
    def control_callback(self, msg):
        if msg.frequency == 1 and msg.switch_control:
            if self.first_pose_stamped is None:
                self.pending_control_msg = msg
                return
            self.process_control_command(msg)
    
    def process_control_command(self, msg):
        try:
            if self.first_pose_stamped is None or self.current_robot_state is None:
                return
            
            if not hasattr(self.current_robot_state, 'machine_pose') or len(self.current_robot_state.machine_pose) < 3:
                return
            process = String()
            process.data = 'move_first'
            pose_msg = PoseStamped()
            velocity_msg = Float64()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'world'
            
            pose_msg.pose.position.x = self.first_pose_stamped.pose.position.x - 116.963
            pose_msg.pose.position.y = self.first_pose_stamped.pose.position.y - 25.52
            pose_msg.pose.position.z = self.first_pose_stamped.pose.position.z - 350.0
            pose_msg.pose.orientation = self.first_pose_stamped.pose.orientation
            velocity_msg.data = 60.0
            self.publisher.publish(pose_msg)
            self.arm_velocity_publisher.publish(velocity_msg)
            next_control_msg = ControlMsg()
            next_control_msg.switch_control = True
            next_control_msg.frequency = msg.frequency + 1
            self.control_publisher.publish(next_control_msg)
            self.process_pub.publish(process)
            self.first_pose_stamped = None
        except Exception as e:
            error = String()
            error.data = f"第一次移动失败: {str(e)}"
            self.error_pub.publish(error)
def main(args=None):
    rclpy.init(args=args)
    node = FirstScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()