# launch/infrared_distance_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明可配置参数
        DeclareLaunchArgument('sensor1_host', default_value='192.168.1.202',
                              description='第一个传感器的网关IP'),
        DeclareLaunchArgument('sensor1_port', default_value='4198',
                              description='第一个传感器的网关端口'),
        DeclareLaunchArgument('sensor2_host', default_value='192.168.1.201',
                              description='第二个传感器的网关IP'),
        DeclareLaunchArgument('sensor2_port', default_value='4197',
                              description='第二个传感器的网关端口'),
        DeclareLaunchArgument('timeout', default_value='5',
                              description='Modbus 连接超时（秒）'),
        DeclareLaunchArgument('publish_rate', default_value='10.0',
                              description='发布频率（Hz）'),

        DeclareLaunchArgument('inclination_host', default_value='192.168.1.203'),
        DeclareLaunchArgument('inclination_port', default_value='4199'),
        DeclareLaunchArgument('inclination_slave', default_value='5'),
        DeclareLaunchArgument('inclination_rate', default_value='10.0'),
        # 启动节点
        Node(
            package='sensor',      # 替换为你的包名
            executable='infrared_distance_node',  # 与 setup.py 中 entry_points 一致
            name='infrared_distance_node',
            output='screen',
            parameters=[{
                'sensor1_host': LaunchConfiguration('sensor1_host'),
                'sensor1_port': LaunchConfiguration('sensor1_port'),
                'sensor2_host': LaunchConfiguration('sensor2_host'),
                'sensor2_port': LaunchConfiguration('sensor2_port'),
                'timeout': LaunchConfiguration('timeout'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }]
        ),
        Node(
            package='sensor',
            executable='inclination_sensor_node',
            name='inclination_sensor_node',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('inclination_host'),
                'port': LaunchConfiguration('inclination_port'),
                'slave': LaunchConfiguration('inclination_slave'),
                'timeout': LaunchConfiguration('timeout'),
                'publish_rate': LaunchConfiguration('inclination_rate'),
            }]
        )
    ])