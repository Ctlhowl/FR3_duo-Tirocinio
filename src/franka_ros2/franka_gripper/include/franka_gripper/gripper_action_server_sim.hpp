#pragma once

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/gripper_command.hpp>

#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/homing.hpp"
#include "franka_msgs/action/move.hpp"

namespace franka_gripper {

template <typename T>
bool resultIsReady(std::future<T>& t, std::chrono::nanoseconds timeout) {
  return t.wait_for(timeout) == std::future_status::ready;
}

class GripperActionServerSim : public rclcpp::Node {
 public:
  using Homing = franka_msgs::action::Homing;
  using Move = franka_msgs::action::Move;
  using Grasp = franka_msgs::action::Grasp;
  using GripperCommand = control_msgs::action::GripperCommand;

  explicit GripperActionServerSim(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  enum class SimTask { simHoming, simMove, simGrasp, simGripperCommand };

  static std::string getSimTaskName(SimTask task) {
    switch (task) {
      case SimTask::simHoming:
        return {"Homing"};
      case SimTask::simMove:
        return {"Moving"};
      case SimTask::simGrasp:
        return {"Grasping"};
      case SimTask::simGripperCommand:
        return {"GripperCommand"};
      default:
        throw std::invalid_argument("getTaskName is not implemented for this case");
    }
  };

  // Publisher verso Coppelia e Subscriber per il feedback
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_commands_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
  
  // Action Servers
  rclcpp_action::Server<Homing>::SharedPtr homing_server_;
  rclcpp_action::Server<Move>::SharedPtr move_server_;
  rclcpp_action::Server<Grasp>::SharedPtr grasp_server_;
  rclcpp_action::Server<GripperCommand>::SharedPtr gripper_command_server_;

  std::mutex state_mutex_;
  double current_width_{0.0};
  double max_width_{0.08}; // 80mm totale

  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::nanoseconds future_wait_timeout_{0};

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void updateAndPublishCommand(double target_width);
  void publishGripperState();

  // Action Handlers
  rclcpp_action::GoalResponse handleGoal(SimTask task);
  rclcpp_action::CancelResponse handleCancel(SimTask task);

  void executeHoming(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Homing>>& goal_handle);
  void executeMove(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>>& goal_handle);
  void executeGrasp(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Grasp>>& goal_handle);
  void executeGripperCommand(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>& goal_handle);

  template <typename T>
  void executeCommand(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle, SimTask task, double target_width);

  template <typename T>
  void publishFeedback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle);
};

}  // namespace franka_gripper