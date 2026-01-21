#ifndef SINGLE_ARM__GRIPPER_ACTION_SERVER_SIM_HPP_
#define SINGLE_ARM__GRIPPER_ACTION_SERVER_SIM_HPP_

#include <stdexcept>
#include <mutex>
#include <future>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include <control_msgs/action/gripper_command.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/homing.hpp>
#include <franka_msgs/action/move.hpp>
#include <std_srvs/srv/trigger.hpp>


// Versione semplificata dello stato
namespace franka {
  struct GripperState {
    double width;
    double max_width;
    bool is_grasped;
  };
}

namespace single_arm {

template <typename T>
bool resultIsReady(std::future<T>& t, std::chrono::nanoseconds timeout){
  return t.wait_for(timeout) == std::future_status::ready;
}

class GripperActionServerSim : public rclcpp::Node {
 public:
  using Homing = franka_msgs::action::Homing;
  using GoalHandleHoming = rclcpp_action::ServerGoalHandle<Homing>;
  using Move = franka_msgs::action::Move;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;
  using Grasp = franka_msgs::action::Grasp;
  using GoalHandleGrasp = rclcpp_action::ServerGoalHandle<Grasp>;
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;
  using Trigger = std_srvs::srv::Trigger;

  explicit GripperActionServerSim();
 
 private:
  enum class SimTask { simHoming, simMove, simGrasp, simGripperCommand };
  static std::string getTaskName(SimTask task);

  // Parametri
  const double default_speed_ = 0.1;
  const double max_gripper_width_ = 0.08; 

  // Action Servers & Services
  rclcpp_action::Server<Homing>::SharedPtr homing_server_;
  rclcpp_action::Server<Move>::SharedPtr move_server_;
  rclcpp_action::Server<Grasp>::SharedPtr grasp_server_;
  rclcpp_action::Server<GripperCommand>::SharedPtr gripper_command_server_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;

  // Comunicazione Coppelia
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr coppelia_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr coppelia_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex gripper_sim_state_mutex_;
  franka::GripperState current_gripper_state_;
  std::vector<std::string> joint_names_;
  std::chrono::nanoseconds future_wait_timeout_{0};

  // Metodi core
  void onCoppeliaJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publishGripperState();
  void sendToCoppelia(double target_width);
  void stopServiceCallback(const std::shared_ptr<Trigger::Response>& response);

  rclcpp_action::CancelResponse handleCancel(SimTask task);
  rclcpp_action::GoalResponse handleGoal(SimTask task);

  void executeHoming(const std::shared_ptr<GoalHandleHoming>& goal_handle);
  void executeMove(const std::shared_ptr<GoalHandleMove>& goal_handle);
  void executeGrasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle);
  void onExecuteGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle);
  
  void publishGripperCommandFeedback(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle);

  template <typename T>
  bool waitForTarget(double target, const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle) {
    constexpr double threshold = 1e4;
    auto start_time = this->now();
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) return false;
      {
        std::lock_guard<std::mutex> lock(gripper_sim_state_mutex_);
        if (std::abs(current_gripper_state_.width - target) < threshold) return true;
      }
      if ((this->now() - start_time).seconds() > 5.0) {
        RCLCPP_ERROR(this->get_logger(), "Timeout simulazione: target %.3f non raggiunto", target);
        return false;
      }
      loop_rate.sleep();
    }
    return false;
  }

  template <typename T>
  void executeCommand(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle, SimTask task, const std::function<bool()>& command_handler) {
    const auto taskName = getTaskName(task);
    auto command_execution_thread = withResultGenerator<T>(command_handler);
    std::future<std::shared_ptr<typename T::Result>> result_future = std::async(std::launch::async, command_execution_thread);

    while (!resultIsReady(result_future, future_wait_timeout_) && rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        sendToCoppelia(current_gripper_state_.width); // Stop immediato
        goal_handle->canceled(std::make_shared<typename T::Result>());
        return;
      }
      publishGripperWidthFeedback(goal_handle);
    }

    if (rclcpp::ok()) {
      auto result = result_future.get();
      result->success ? goal_handle->succeed(result) : goal_handle->abort(result);
    }
  }

  template <typename T>
  auto withResultGenerator(const std::function<bool()>& command_handler) -> std::function<std::shared_ptr<typename T::Result>()> {
    return [command_handler]() {
      auto result = std::make_shared<typename T::Result>();
      try {
        result->success = command_handler();
      } catch (const std::exception& e) {
        result->success = false;
      }
      return result;
    };
  }

  template <typename T>
  void publishGripperWidthFeedback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle) {
    auto feedback = std::make_shared<typename T::Feedback>();
    std::lock_guard<std::mutex> guard(gripper_sim_state_mutex_);
    feedback->current_width = current_gripper_state_.width;
    goal_handle->publish_feedback(feedback);
  }
};

} // namespace single_arm
#endif