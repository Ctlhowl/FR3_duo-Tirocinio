#pragma once

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/gripper_command.hpp"

#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/move.hpp"

namespace single_arm {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripper = rclcpp_action::ServerGoalHandle<GripperCommand>;

class GripperBridgeController : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // Server che "ascolta" MoveIt
  rclcpp_action::Server<GripperCommand>::SharedPtr moveit_action_server_;

  // Client che "parlano" con l'hardware o simulatore
  rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr gripper_grasp_client_;
  rclcpp_action::Client<franka_msgs::action::Move>::SharedPtr gripper_move_client_;

  // Logica Action Server
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const GripperCommand::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGripper> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle);

  // Esecutori
  void call_hardware_grasp(const std::shared_ptr<GoalHandleGripper> goal_handle);
  void call_hardware_move(const std::shared_ptr<GoalHandleGripper> goal_handle, double position);

  double closing_threshold_ = 0.01;
};

} // namespace single_arm