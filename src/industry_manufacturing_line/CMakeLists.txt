cmake_minimum_required(VERSION 3.8)
project(industry_manufacturing_line)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(apriltag_ros REQUIRED)
find_package(rtabmap_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_core REQUIRED)
find_package(trajectory_msgs REQUIRED)

# ========== 自定义消息/服务 ==========
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectionObject.msg"
  "msg/TaskFeedback.msg"
  "srv/GraspObject.srv"
  "srv/NavigateToStation.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ========== 可执行文件 ==========
add_executable(minimal_node
  src/main.cpp)
ament_target_dependencies(minimal_node
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
)
target_link_libraries(minimal_node "${cpp_typesupport_target}")

# 导航节点
add_executable(navigation_node
  src/Nodes/navigation_node.cpp)
ament_target_dependencies(navigation_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  std_msgs
)
target_link_libraries(navigation_node "${cpp_typesupport_target}")

# 机械臂节点
add_executable(manipulation_node
  src/Nodes/manipulation_node.cpp)
ament_target_dependencies(manipulation_node
  rclcpp
  rclcpp_lifecycle
  moveit_ros_planning_interface
  moveit_ros_move_group
  std_msgs
  geometry_msgs
)
target_link_libraries(manipulation_node "${cpp_typesupport_target}")


# 主控制器节点
add_executable(robot_controller_node
  src/Nodes/robot_controller_node.cpp)
ament_target_dependencies(robot_controller_node
  rclcpp
  rclcpp_lifecycle
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_core
  std_msgs
  geometry_msgs
)
target_link_libraries(robot_controller_node "${cpp_typesupport_target}")


# ========== 安装 ==========
install(TARGETS
  minimal_node
  navigation_node
  manipulation_node
  robot_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  msg
  srv
  DESTINATION share/${PROJECT_NAME}/
)

# ========== 测试 ==========
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
