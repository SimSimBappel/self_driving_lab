cmake_minimum_required(VERSION 3.5)
project(mtc_pour)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# Find packages in ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_marker_tools REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

# Create a shared library
add_library(mtc_pour_into SHARED
  src/pour_into.cpp
)

# Include directories
include_directories(
  include
)

# Include directories and dependencies for mtc_pour_into
target_include_directories(mtc_pour_into PUBLIC
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(mtc_pour_into
  rclcpp
  rviz_marker_tools
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  tf2_eigen
  geometry_msgs
  shape_msgs
)

# Link MoveIt libraries
target_link_libraries(mtc_pour_into
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
  # ${moveit_task_constructor_core_INCLUDE_DIRS}  # Include directories from MoveIt Task Constructor
)

# Install targets and directories
install(TARGETS mtc_pour_into
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config meshes
  DESTINATION share/${PROJECT_NAME}
)

# Export information for other packages
ament_export_dependencies(rclcpp rviz_marker_tools moveit_core moveit_ros_planning_interface moveit_task_constructor_core tf2_eigen)
ament_export_include_directories(include)

ament_package()
