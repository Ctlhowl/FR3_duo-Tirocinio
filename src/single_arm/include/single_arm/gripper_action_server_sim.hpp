#ifndef SINGLE_ARM__GRIPPER_ACTION_SERVER_SIM_HPP_
#define SINGLE_ARM__GRIPPER_ACTION_SERVER_SIM_HPP_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include <control_msgs/action/gripper_command.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/homing.hpp>
#include <franka_msgs/action/move.hpp>


#include <std_srvs/srv/trigger.hpp>

namespace single_arm {

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
  /// Describes the different tasks. Each task corresponds to one action server
  enum class SimTask { simHoming, simMove, simGrasp, simGripperCommand };
  static std::string getTaskName(SimTask task) {
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
        throw std::invalid_argument("[ERRORE]: getTaskName non implementato per questo caso");
    }
  };

  rclcpp_action::Server<Homing>::SharedPtr homing_server_;
  rclcpp_action::Server<Move>::SharedPtr move_server_;
  rclcpp_action::Server<Grasp>::SharedPtr grasp_server_;
  rclcpp_action::Server<GripperCommand>::SharedPtr gripper_command_server_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr coppelia_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;

  std::mutex state_mutex_;
  double current_width_ = 0.0;

  // Functions
  rclcpp_action::CancelResponse handleCancel(SimTask task);
  rclcpp_action::GoalResponse handleGoal(SimTask task);

  void stopServiceCallback(const std::shared_ptr<Trigger::Response>& response);
  void executeHoming(const std::shared_ptr<GoalHandleHoming>& goal_handle);
  void executeMove(const std::shared_ptr<GoalHandleMove>& goal_handle);
  void executeGrasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle);
  void executeGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle);

  void feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Utility function to wait until the simulated gripper reaches the target width
  template <typename T>
  bool waitForTarget(double target, const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle) {
    constexpr double threshold = 1e-4;
    auto start_time = this->now();
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (std::abs(current_width_ - target) < threshold) {
                return true;
            }
        }

        if ((this->now() - start_time).seconds() > 5.0) {
            RCLCPP_ERROR(this->get_logger(), 
            "Timeout raggiungimento target simulato, current_width_ :%.3f  , target: %.3f", current_width_, target);
            return false;
        }

        loop_rate.sleep();
    }
    return false;
}
};

}  // namespace single_arm

#endif // SINGLE_ARM__GRIPPER_ACTION_SERVER_SIM_HPP_