cmake_minimum_required(VERSION 3.8)
project(pgsql_services)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-lpqxx -lpq)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pgsql_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_library(PQXX_LIB pqxx)
find_library(PQ_LIB pq)

include_directories(
  include
)

add_executable(database_service src/database_service.cpp)

target_link_libraries(database_service  
 ${catkin_LIBRARIES}
 ${PQXX_LIB} 
 ${PQ_LIB}
)

ament_target_dependencies(database_service rclcpp pgsql_interfaces geometry_msgs std_msgs ament_index_cpp)

#target_link_libraries(database_service PRIVATE pqxx)

install(
  TARGETS database_service
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

# file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

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
