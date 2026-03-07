from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable



def generate_launch_description():
    return LaunchDescription([
 
        Node(
            package='unitreemotor',
            executable='unitree_motor_node',
            name='unitree_motor_node',
            output='screen'
        ),
    ])

