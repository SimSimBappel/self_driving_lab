cmake_minimum_required(VERSION 3.22)
project(omtp_factory_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(std_msgs REQUIRED)

find_package(tf2 REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(behavior_tree_ros2_actions REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

add_executable(move_demo src/move_demo.cpp)
ament_target_dependencies(move_demo rclcpp std_msgs moveit_ros_planning_interface)

# add_executable(move_robot_server src/move_robot_server.cpp)
# ament_target_dependencies(move_robot_server behaviortree_ros2 rclcpp std_msgs moveit_ros_planning_interface tf2 geometry_msgs behavior_tree_ros2_actions) 

include_directories(include)

install(TARGETS
  move_demo
  # simple_move_pub
  # move_robot_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()

install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
