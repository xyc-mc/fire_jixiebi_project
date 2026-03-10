      ______      __       ___________       ___________       _________        __         __    
     |     \ \   |  |\    |   ________|\    |   ________|\    |   ____  \\     |  |\      |  |\  
     |  |\  \ \  |  | |   |  |\ _______\|   |  |\ _______\|   |  |\___\  \\    |  | |     |  | | 
     |  | \  \ \ |  | |   |  |_|___         |  |_|___         |  | |   |  ||   |  | |     |  | | 
     |  | |\  \ \|  | |   |   _____|\       |   _____|\       |  |_|__/  / /   |  | |     |  | | 
     |  | | \  \ |  | |   |  | _____\|      |  | _____\|      |  |______/ /    |  | |     |  | | 
     |  | |  \  \|  | |   |  |_|______      |  |_|______      |  |\_____\/      \  \|____/  / \| 
     |__| |   \_____| |   |___________|\    |___________|\    |__| |             \_________/ /   
      \__\|    \_____\|    \___________\|    \___________\|    \__\|              \________\/    
                                            
# 项目简介

## 项目名称：机械臂火警探测器拆装系统

    - 该项目旨在通过控制机械臂去对天花板上的火警探头进行拆装，通过相机进行识别，相机安装在机械臂上。
    - 该项目分为两部分，第一部分是机械臂的控制，第二部分是天花板上火警探头的识别。
    - 该项目使用ros2进行开发
    - 该项目使用机械臂、相机和传感器等程序基于python进行开发，末端执行电机基于C++进行开发

## 项目信息

    - 开发人员：WZH，XYC
    - 开发时间：2025.11-2026.1
    - 开发环境：Ubuntu22.04 + ROS2 Humble
    - 开发语言：Python3, C++
    - 开发工具：Visual Studio Code
 
## 硬件设备
    
    - 机械臂：Elite EC66
    - 相机：微视VSensor PMR0700
    - 其他传感器：倾角传感器、红外线测距传感器
    - 末端执行器：宇树GO-M8010

# 项目结构
    
    ├── fire_detector
    │   └── src
    │       │── area_scan_camera                  - 相机控制功能包
    |       |   │── area_scan_camera              - 相机控制程序     
    |       |   |   │── __init__.py
    |       |   |   │── camera_control.py
    |       |   |   │── program_entrance.py
    |       |   |   └── VSensorSDK.py             - 相机SDK    
    |       |   │── launch
    │       |   │   └── camera_control.launch.py  - 相机节点启动文件
    │       │   │──package.xml
    │       │   │──setup.py
    │       │   └──setup.cfg
    │       |
    │       │── base_msgs                         - 基础消息包
    |       |   │── msgs
    |       |   │   │──ControlMsg.msg             - 控制消息
    |       |   |   └──DataSend.msg               - 上位机通讯消息
    |       |   │──CMakeLists.txt
    |       |   └──package.xml
    │       |
    │       │── elite_arm_controller
    |       |   │── elite_arm_controller
    |       |   │   │── __init__.py
    |       |   │   |── movecontroll.py   
    |       |   │   │── first_scan.py
    |       |   │   |── second_scan.py
    |       |   │   |── uninstall.py
    |       |   │   └── install.py 
    |       |   │── config
    |       |   │── launch
    |       |   │   └── execute.launch.py
    │       │   │──package.xml
    │       │   │──setup.py
    │       │   └──setup.cfg
    │       |
    │       |── elite_arm_driver
    │       │   └── ...
    │       |
    │       |── elite_description
    │       │   └── ...
    │       |
    │       |── elite_msgs
    │       │   └── ...
    │       |
    │       |── handeye_coord_transformer
    │       │   |── handeye_coord_transformer
    |       |   │   └── ...
    |       |   │── launch
    |       |   │   └── ...
    |       |   │── config
    |       |   │   └── ...
    │       │   │──package.xml
    │       │   │──setup.py
    │       │   └──setup.cfg
    │       |
    │       |── point_rgb_address
    │       │   └── ...
    │       |
    │       |── sensor
    │       │   └── ...
    │       |
    │       |── start
    │       │   └── ...
    │       |
    │       |── unitreemotor  
    │       │   └── ...
    │       |
    │       └── unitreemotor_msgs
    │           └── ...
    │—— first_use.sh
    └── README.md
   
# 项目安装步骤
> 在终端输入以下命令，安装该项目
> 1. 电脑需要安装ROS2 Humble，推荐小鱼社区的安装教程
>   - wget http://fishros.com/install -O fishros && . fishros
> 2. 安装依赖库
>   1. 安装 python3-pip
>      - sudo apt install python3-pip
>   2. 安装 Python 依赖项
>      - pip3 install elirobots transforms3d pytest rosdepc opencv open3d
>   3. 安装 control-msgs
>      - sudo apt install ros-humble-control-msgs
>   4. 安装 ROS2 控制器
>      - sudo apt install ros-humble-controller
>   5. 安装 yolo 依赖
>      - pip3 install ultralytics

# 编译步骤
> - 在终端输入以下命令，编译该项目
> - 将程序克隆到本地:git clone https://github.com/xyc-mc/fire_jixiebi_project
> - 进入到项目目录:cd fire_jixiebi_project
> - 编译项目:colcon build
> - 编译成功后，在终端输入以下命令启动项目
> - . install/setup.bash
> - ros2 launch start start.launch.py
> - 程序挂挂起后终端会输出节点信息，此时程序已经运行,等待启动指令
> - 给机械臂发送启动指令:ros2 topic pub /start std_msgs/msg/String {"data: 'start'"}

    
