#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <control_msgs/action/gripper_command.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <franka_gripper/gripper_action_server_sim.hpp>

namespace franka_gripper {

GripperActionServerSim::GripperActionServerSim(const rclcpp::NodeOptions& options) : Node("franka_gripper_sim_node", options) {
  
  this->declare_parameter("joint_names", std::vector<std::string>{"fr3_finger_joint1", "fr3_finger_joint2"});
  
  // Publisher comandi verso Coppelia (JointState come richiesto)
  this->joint_commands_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/coppelia/gripper/joint_commands", 1);
  this->joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 1);

  // Subscriber feedback da Coppelia
  this->joint_states_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/coppelia/gripper/joint_states", 10,
      std::bind(&GripperActionServerSim::onJointState, this, std::placeholders::_1));

  future_wait_timeout_ = std::chrono::milliseconds(100);
  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(33),
    [this]() { return publishGripperState(); });

  // Inizializzazione Action Servers
  const auto simHomingTask = SimTask::simHoming;
  this->homing_server_ = rclcpp_action::create_server<Homing>(
      this, "~/homing",
      [this, simHomingTask](auto /*uuid*/, auto /*goal*/) { return handleGoal(simHomingTask); },
      [this, simHomingTask](const auto& /*goal_handle*/) { return handleCancel(simHomingTask); },
      [this](const auto& goal_handle) {
        return std::thread{[goal_handle, this]() { executeHoming(goal_handle); }}.detach();
      });

  const auto simMoveTask = SimTask::simMove;
  this->move_server_ = rclcpp_action::create_server<Move>(
      this, "~/move",
      [this, simMoveTask](auto /*uuid*/, auto /*goal*/) { return handleGoal(simMoveTask); },
      [this, simMoveTask](const auto& /*goal_handle*/) { return handleCancel(simMoveTask); },
      [this](const auto& goal_handle) {
        return std::thread{[goal_handle, this]() { executeMove(goal_handle); }}.detach();
      });

  const auto simGraspTask = SimTask::simGrasp;
  this->grasp_server_ = rclcpp_action::create_server<Grasp>(
      this, "~/grasp",
      [this, simGraspTask](auto /*uuid*/, auto /*goal*/) { return handleGoal(simGraspTask); },
      [this, simGraspTask](const auto& /*goal_handle*/) { return handleCancel(simGraspTask); },
      [this](const auto& goal_handle) {
        return std::thread{[goal_handle, this]() { executeGrasp(goal_handle); }}.detach();
      });
  
  const auto simGripperCommandTask = SimTask::simGripperCommand;
  this->gripper_command_server_ = rclcpp_action::create_server<GripperCommand>(
    this, "~/gripper_action",
    [this, simGripperCommandTask](auto, auto) { return handleGoal(simGripperCommandTask); },
    [this, simGripperCommandTask](const auto&) { return handleCancel(simGripperCommandTask); },
    [this](const auto& goal_handle) {
        std::thread([this, goal_handle]() { executeGripperCommand(goal_handle); }).detach();
    });

  RCLCPP_INFO(this->get_logger(), "Simulated Franka Gripper Bridge pronto.");
}

void GripperActionServerSim::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "fr3_finger_joint1") {
      current_width_ = msg->position[i] * 2.0;
    }
  }
}

void GripperActionServerSim::updateAndPublishCommand(double target_width) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name = {"fr3_finger_joint1", "fr3_finger_joint2"};
  msg.position = {target_width / 2.0, target_width / 2.0};
  joint_commands_publisher_->publish(msg);
}

// Implementazione specifica delle azioni
void GripperActionServerSim::executeHoming(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Homing>>& goal_handle) {
  executeCommand(goal_handle, SimTask::simHoming, max_width_);
}

void GripperActionServerSim::executeMove(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>>& goal_handle) {
  executeCommand(goal_handle, SimTask::simMove, goal_handle->get_goal()->width);
}

void GripperActionServerSim::executeGrasp(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Grasp>>& goal_handle) {
  executeCommand(goal_handle, SimTask::simGrasp, goal_handle->get_goal()->width);
}

void GripperActionServerSim::executeGripperCommand(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>& goal_handle) {
    
    const auto goal = goal_handle->get_goal();
    // In GripperCommand, la posizione è per singolo dito, quindi moltiplichiamo per 2
    double target_width = 2.0 * goal->command.position;
    
    auto result = std::make_shared<GripperCommand::Result>();
    updateAndPublishCommand(target_width); // Invia a CoppeliaSim

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    result->position = target_width / 2.0;
    result->reached_goal = true;
    goal_handle->succeed(result);
}

template <typename T>
void GripperActionServerSim::executeCommand(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle, SimTask task, double target_width) {
  const auto simTaskName = getSimTaskName(task);

  RCLCPP_INFO(this->get_logger(), "Gripper %s a width: %f", simTaskName.c_str(), target_width);

  auto result = std::make_shared<typename T::Result>();
  updateAndPublishCommand(target_width);

  // Loop di feedback
  rclcpp::Rate loop_rate(10);
  auto start_time = this->now();
  bool reached = false;

  while (rclcpp::ok() && (this->now() - start_time) < rclcpp::Duration(3, 0)) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }
    
    publishFeedback(goal_handle);
    
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (std::abs(current_width_ - target_width) < 0.002) {
        reached = true;
        break;
      }
    }
    loop_rate.sleep();
  }

  result->success = reached;
  if (reached) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
}

template <typename T>
void GripperActionServerSim::publishFeedback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle) {
  auto feedback = std::make_shared<typename T::Feedback>();
  std::lock_guard<std::mutex> lock(state_mutex_);
  feedback->current_width = current_width_;
  goal_handle->publish_feedback(feedback);
}

rclcpp_action::GoalResponse GripperActionServerSim::handleGoal(SimTask) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperActionServerSim::handleCancel(SimTask) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperActionServerSim::publishGripperState() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  sensor_msgs::msg::JointState joint_states;
  joint_states.header.stamp = this->now();
  joint_states.name = {"fr3_finger_joint1", "fr3_finger_joint2"};
  joint_states.position = {current_width_ / 2, current_width_ / 2};
  joint_states.velocity = {0.0, 0.0};
  joint_states.effort = {0.0, 0.0};
  joint_states_publisher_->publish(joint_states);
}

} // namespace franka_gripper_sim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(franka_gripper::GripperActionServerSim)