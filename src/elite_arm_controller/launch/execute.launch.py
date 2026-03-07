from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        # 机械臂控制节点
        Node(
            package='elite_arm_controller',
            executable='pose_to_joint_controller',
            name='pose_to_joint_controller',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        
        # 扫描节点
        Node(
            package='elite_arm_controller',
            executable='first_scan',
            name='first_scan',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        # 安装扫描节点
        Node(
            package='elite_arm_controller',
            executable='install_node',
            name='install_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        # 拆卸扫描节点
        Node(
            package='elite_arm_controller',
            executable='uninstall_node',
            name='uninstall_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    ])
