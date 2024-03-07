cmake_minimum_required(VERSION 3.8)
project(behavior_tree_ros2_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(THIS_PACKAGE_DEPS 
    behaviortree_ros2
    std_msgs )


set(action_files
    "action/Sleep.action"
    "action/FindArucoTag.action"
    "action/Home.action"
    "action/Gripper.action"
    "action/GripperJoint.action"
    "action/ArmMoveJoints.action"
    "action/ArmMovePose.action"
    "action/ArmMoveRelativePose.action"
    "action/ArmMoveToFrame.action"
    # "action/MoveGroupSequence.action"
    # "action/Pickup.action"
    # "action/Place.action"
    # "action/LocalPlanner.action"
    # "action/GlobalPlanner.action"
    # "action/HybridPlanner.action"
  )
  
  rosidl_generate_interfaces(${PROJECT_NAME}
    # ${msg_files}
    # ${srv_files}
    ${action_files}
    DEPENDENCIES
      action_msgs
      builtin_interfaces
      std_msgs
      geometry_msgs
      # sensor_msgs
      # shape_msgs
      # trajectory_msgs
  )

ament_export_dependencies(rosidl_default_runtime)

# add_library(sleep_plugin SHARED src/sleep_action.cpp)
# target_compile_definitions(sleep_plugin PRIVATE  BT_PLUGIN_EXPORT )
# ament_target_dependencies(sleep_plugin ${THIS_PACKAGE_DEPS})

# add_library(home_plugin SHARED src/home_action.cpp)
# target_compile_definitions(home_plugin PRIVATE  BT_PLUGIN_EXPORT )
# ament_target_dependencies(home_plugin ${THIS_PACKAGE_DEPS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
