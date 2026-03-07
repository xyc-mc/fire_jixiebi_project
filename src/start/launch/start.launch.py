from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():

    point_rgb_address_dir = get_package_share_directory("point_rgb_address")
    area_scan_camera = get_package_share_directory('area_scan_camera')
    elite_arm_driver = get_package_share_directory('elite_arm_driver')
    elite_arm_controller = get_package_share_directory('elite_arm_controller')
    handeye_coord_transformer = get_package_share_directory('handeye_coord_transformer')
    unitree_motor = get_package_share_directory('unitreemotor')


    return LaunchDescription([
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(area_scan_camera,"launch","camera_control.launch.py")
            ),
        ),

        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(point_rgb_address_dir,"launch","address_control.launch.py")
            ),
        ),

        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(elite_arm_controller,"launch","execute.launch.py")
            ),
        ),

        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(handeye_coord_transformer,"launch","handeye_system.launch.py")
            ),
        ),   

        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(elite_arm_driver,"launch","bringupec66.launch.py")
            ),
        ),

        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(unitree_motor,"launch","motor.launch.py")
            ),
        ),
        
    ])