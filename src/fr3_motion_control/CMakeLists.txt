cmake_minimum_required(VERSION 3.8)
project(fr3_motion_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(move_fr3_node src/move_fr3_node.cpp)

ament_target_dependencies(move_fr3_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

install(TARGETS
  move_fr3_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
