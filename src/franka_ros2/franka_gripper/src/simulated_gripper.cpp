#include "franka_gripper/simulated_gripper.hpp"

namespace franka_gripper {

SimulatedGripper::SimulatedGripper(rclcpp::Node* node) : node_(node) {
  node_->get_parameter("joint_names", joint_names_);
  //joint_names_ = {"fr3_finger_joint1", "fr3_finger_joint2"};

  // Topic verso CoppeliaSim
  cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("~/joint_commands", 10);
  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "~/coppelia_states", 10, 
      std::bind(&SimulatedGripper::jointStateCallback, this, std::placeholders::_1));
  
  current_state_.max_width = 0.08; // Valore standard Panda
}

void SimulatedGripper::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Cerchiamo i giunti corretti nel messaggio ricevuto
    double w = 0.0;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "fr3_finger_joint1" || msg->name[i] == "fr3_finger_joint2") {
        w += msg->position[i] / 2;
      }
    }
    
    current_state_.width = w;
    current_state_.is_grasped = (w < 0.075 && w > 0.001);
}

bool SimulatedGripper::homing() { 
  return move(0.08, 0.1); 
}

bool SimulatedGripper::move(double width, double /*speed*/) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();
  msg.name = joint_names_;
  msg.position.push_back(width / 2.0);
  msg.position.push_back(width / 2.0);
  cmd_pub_->publish(msg);

  return true;
}

bool SimulatedGripper::grasp(double width, double speed, double /*force*/, double /*in*/, double /*out*/) {
  return move(width, speed);
}

bool SimulatedGripper::stop() { return true; }

franka::GripperState SimulatedGripper::readOnce() {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_;
}

} // namespace franka_gripper