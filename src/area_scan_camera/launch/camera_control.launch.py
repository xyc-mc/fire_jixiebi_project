from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    start_pkg_share = get_package_share_directory('start')
    config_path = os.path.join(start_pkg_share, 'config', 'init.yaml')
    return LaunchDescription([
        Node(
            package='area_scan_camera',
            executable='camera_control',
            name='area_scan_camera',
            output='screen',
            parameters=[config_path]
        ),

        Node(
            package='area_scan_camera',
            executable='program_entrance',
            name='area_scan_camera1',
            output='screen',
            parameters=[config_path]
        )
    ])