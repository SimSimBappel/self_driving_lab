cmake_minimum_required(VERSION 3.8)
project(behavior_tree_ros2_rob8)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(camera REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behavior_tree_ros2_actions REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pgsql_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(THIS_PACKAGE_DEPS 
    behaviortree_ros2
    std_msgs
    std_srvs
    geometry_msgs
    tf2_geometry_msgs
    tf2
    behavior_tree_ros2_actions
    franka_msgs
    pgsql_interfaces )

add_executable(test_load 
    src/sleep_action.cpp
    src/gripper_behaviors.cpp
    src/arm_behaviors.cpp
    src/camera_behaviors.cpp
    src/test_load_and_make_xml_files.cpp)
    
 ament_target_dependencies(test_load ${THIS_PACKAGE_DEPS})

 add_executable(behavior_server 
    src/sleep_action.cpp
    src/gripper_behaviors.cpp
    src/arm_behaviors.cpp
    src/camera_behaviors.cpp
    src/behavior_server.cpp)
    
 ament_target_dependencies(behavior_server ${THIS_PACKAGE_DEPS})

  add_executable(test_client 
    src/sleep_action.cpp
    src/gripper_behaviors.cpp
    src/arm_behaviors.cpp
    src/camera_behaviors.cpp
    src/database_behaviors.cpp
    src/test_client.cpp)
    
  ament_target_dependencies(test_client ${THIS_PACKAGE_DEPS})

add_library(sleep2_plugin SHARED src/sleep_action.cpp)
target_compile_definitions(sleep2_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(sleep2_plugin ${THIS_PACKAGE_DEPS})

# add_library(home_plugin SHARED src/home_action.cpp)
# target_compile_definitions(home_plugin PRIVATE  BT_PLUGIN_EXPORT )
# ament_target_dependencies(home_plugin ${THIS_PACKAGE_DEPS})

install(TARGETS
  test_client
  test_load
  behavior_server
  # sleep_client_dyn
  
  # sleep_server
  # home_plugin
  sleep2_plugin
  # subscriber_test
  DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY bt_xml DESTINATION share/${PROJECT_NAME})

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
