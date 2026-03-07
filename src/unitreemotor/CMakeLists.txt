# 1. 版本要求
cmake_minimum_required(VERSION 3.8)
project(unitreemotor)

# 2. 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) 

# 3. 编译警告设置（可选，但对新手有帮助）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 4. 添加优化选项（可选）
add_compile_options(-O3)

# 5. 查找ROS2依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitreemotor_msgs REQUIRED)


# 6. 包含电机SDK作为子项目
# 这会执行电机SDK的CMakeLists.txt
add_subdirectory(unitree_actuator_sdk)

# 7. 设置包含目录（非常重要！）
include_directories(
  # ROS2节点自己的头文件目录
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  
  # 电机SDK的头文件目录
  ${CMAKE_CURRENT_SOURCE_DIR}/unitree_actuator_sdk/include
  ${CMAKE_CURRENT_SOURCE_DIR}/unitree_actuator_sdk/include/serialPort
  ${CMAKE_CURRENT_SOURCE_DIR}/unitree_actuator_sdk/include/unitreeMotor
)

# 8. 设置库文件目录
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/unitree_actuator_sdk/lib
)

# 9. 平台检测和库选择
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64") 
    message(STATUS "检测到ARM64架构，使用ARM64库")
    set(EXTRA_LIBS libUnitreeMotorSDK_Arm64.so)
else()
    message(STATUS "检测到x86_64架构，使用Linux64库")
    set(EXTRA_LIBS libUnitreeMotorSDK_Linux64.so)
endif()

# 10. 创建您的ROS2节点
add_executable(unitree_motor_node
  src/unitree_motor_node.cpp  # 节点源文件
  # 如果还有其他源文件，在这里添加
  # src/simple_test_node.cpp
)


# 11. 链接库到您的节点
target_link_libraries(unitree_motor_node
  ${EXTRA_LIBS}           # 电机SDK库
  ${rclcpp_LIBRARIES}     # ROS2核心库
  ${std_msgs_LIBRARIES}   # ROS2标准消息库
  ${geometry_msgs_LIBRARIES}  # ROS2几何消息库
  pthread
  rt
)

# 12. 设置ROS2依赖
ament_target_dependencies(unitree_motor_node
  rclcpp
  std_msgs
  geometry_msgs
  unitreemotor_msgs
)

# 13. 安装可执行文件
install(TARGETS unitree_motor_node
  DESTINATION lib/${PROJECT_NAME}
)

# 14. 安装启动文件
install(PROGRAMS
  launch/motor.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# 15. 安装头文件（如果需要其他包使用）
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 16. 安装电机SDK的头文件（可选，如果需要）
install(DIRECTORY unitree_actuator_sdk/include/
  DESTINATION include/${PROJECT_NAME}/unitree_sdk
)

# 17. 安装电机SDK的库文件（可选）
install(FILES 
  unitree_actuator_sdk/lib/${EXTRA_LIBS}
  DESTINATION lib  # 直接安装到 lib/
)


# 18. 导出依赖关系
ament_export_include_directories(
  include
  unitree_actuator_sdk/include
  unitree_actuator_sdk/include/serialPort
  unitree_actuator_sdk/include/unitreeMotor
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
)


# 19. 告诉ROS2构建系统完成配置
ament_package()