cmake_minimum_required(VERSION 3.8)
project(single_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(mimic_node src/mimic_node.cpp)
add_executable(gripper_bridge src/gripper_bridge.cpp)
add_executable(gripper_action_server_sim src/gripper_action_server_sim.cpp)

foreach(target gripper_bridge gripper_action_server_sim)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endforeach()

ament_target_dependencies(mimic_node rclcpp sensor_msgs)
ament_target_dependencies(gripper_bridge rclcpp rclcpp_action sensor_msgs control_msgs std_srvs franka_msgs)
ament_target_dependencies(gripper_action_server_sim rclcpp rclcpp_action sensor_msgs control_msgs std_srvs franka_msgs)

install(TARGETS
  mimic_node
  gripper_bridge
  gripper_action_server_sim
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

ament_package()