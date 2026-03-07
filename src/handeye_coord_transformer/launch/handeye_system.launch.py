from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='handeye_coord_transformer',
            executable='handeye_transform_node',
            name='handeye_transform_node',
            output='screen',
            parameters=[{
                # 这里可以添加需要从启动文件传入的参数
            }]
        ),
        Node(
            package='handeye_coord_transformer',
            executable='plane_normal_to_rpy_node',
            name='plane_normal_to_rpy_node',
            output='screen',
            parameters=[{
                # 这里可以添加需要从启动文件传入的参数
            }]
        )
    ])