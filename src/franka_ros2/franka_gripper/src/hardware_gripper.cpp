#include "franka_gripper/hardware_gripper.hpp"

namespace franka_gripper {

HardwareGripper::HardwareGripper(const std::string& robot_ip) {
  gripper_ = std::make_unique<franka::Gripper>(robot_ip);
}

bool HardwareGripper::homing() { 
  return gripper_->homing();
}

bool HardwareGripper::move(double width, double speed) { 
  return gripper_->move(width, speed); 
}

bool HardwareGripper::grasp(double width, double speed, double force, double inner, double outer) {
  return gripper_->grasp(width, speed, force, inner, outer);
}

bool HardwareGripper::stop() { 
  return gripper_->stop(); 
}

franka::GripperState HardwareGripper::readOnce() { return gripper_->readOnce(); }

} // namespace franka_gripper