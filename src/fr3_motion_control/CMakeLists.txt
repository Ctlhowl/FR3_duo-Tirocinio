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

add_executable(dual_hand_changing src/dual_hand_changing_motion.cpp)
add_executable(start_pose src/start_pose_motion.cpp)

ament_target_dependencies(dual_hand_changing rclcpp moveit_ros_planning_interface geometry_msgs)
ament_target_dependencies(start_pose rclcpp moveit_ros_planning_interface geometry_msgs)

install(TARGETS
  dual_hand_changing
  start_pose
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
