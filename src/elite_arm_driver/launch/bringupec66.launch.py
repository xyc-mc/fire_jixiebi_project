import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # 声明启动参数
    ip_addr_launch_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.1.200',
        description='IP address of the robot arm')
    
    auto_connect_launch_arg = DeclareLaunchArgument(
        'auto_connect',
        default_value='True',
        description='Whether to auto connect to the robot arm')
    
    use_fake_launch_arg = DeclareLaunchArgument(
        'use_fake',
        default_value='False',
        description='Whether to use fake hardware for testing')
    
    timer_period_launch_arg = DeclareLaunchArgument(
        'timer_period',
        default_value=TextSubstitution(text='0.01'),
        description='Timer period for the driver')
    
    start_rviz_launch_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='False',
        description='Whether to start RViz')

    # 获取配置值
    ip_address = LaunchConfiguration('ip_address')
    auto_connect = LaunchConfiguration('auto_connect')
    use_fake = LaunchConfiguration('use_fake')
    timer_period = LaunchConfiguration('timer_period')
    start_rviz = LaunchConfiguration('start_rviz')
    
    # 获取包路径
    elite_description_path = get_package_share_directory('elite_description')
    
    # 创建机器人驱动节点
    bringup_arm_driver = Node(
        package="elite_arm_driver",
        executable="elite",
        name="elite_arm_driver",
        parameters=[{
            'ip_address': ip_address,
            'auto_connect': auto_connect,
            'use_fake': use_fake,
            'timer_period': timer_period}],
        output='screen'
    )
    
    # URDF文件路径（固定为ec66）
    urdf_path = os.path.join(elite_description_path, 'urdf', 'ec66_description_real.urdf')
    
    # 机器人状态发布器节点
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command([
            "xacro", " ", urdf_path])}]
    )
    
    # RViz启动（可选）
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(elite_description_path, 'launch', 'elite_description_ec66.launch.py')
        ]),
        condition=IfCondition(start_rviz)
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(ip_addr_launch_arg)
    ld.add_action(auto_connect_launch_arg)
    ld.add_action(use_fake_launch_arg)
    ld.add_action(timer_period_launch_arg)
    ld.add_action(start_rviz_launch_arg)
    
    # 添加节点
    ld.add_action(bringup_arm_driver)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_launch)
    
    return ld