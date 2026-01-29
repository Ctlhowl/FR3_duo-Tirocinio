#pragma once
#include "franka_gripper/IGripper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>

namespace franka_gripper {

class SimulatedGripper : public IGripper {
public:
  explicit SimulatedGripper(rclcpp::Node* node);
  bool homing() override;
  bool move(double width, double speed) override;
  bool grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer) override;
  bool stop() override;
  franka::GripperState readOnce() override;

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  
  rclcpp::Node* node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  
  std::mutex mutex_;
  franka::GripperState current_state_;
  std::vector<std::string> joint_names_;
};

} // namespace franka_gripper