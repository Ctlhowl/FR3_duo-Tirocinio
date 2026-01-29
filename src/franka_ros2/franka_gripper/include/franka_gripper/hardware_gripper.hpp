#pragma once
#include "franka_gripper/IGripper.hpp"
#include <franka/gripper.h>
#include <memory>

namespace franka_gripper {

class HardwareGripper : public IGripper {
public:
  explicit HardwareGripper(const std::string& robot_ip);
  bool homing() override;
  bool move(double width, double speed) override;
  bool grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer) override;
  bool stop() override;
  franka::GripperState readOnce() override;

private:
  std::unique_ptr<franka::Gripper> gripper_;
};

} // namespace franka_gripper