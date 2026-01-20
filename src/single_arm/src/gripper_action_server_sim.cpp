#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <single_arm/gripper_action_server_sim.hpp>

namespace single_arm {

  GripperActionServerSim::GripperActionServerSim() : rclcpp::Node("franka_gripper_node") {
    // Creazione Action Server per ogni task
    this->stop_service_ = create_service<Trigger>("~/stop",
        [this](std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response) { return stopServiceCallback(std::move(response)); });

    const auto simHomingTask = SimTask::simHoming;
    this->homing_server_ = rclcpp_action::create_server<Homing>(
        this, "~/homing",
        [this, simHomingTask](auto, auto) { return handleGoal(simHomingTask); },
        [this, simHomingTask](const auto&) { return handleCancel(simHomingTask); },
        [this](const auto& goal_handle) {
          return std::thread{[goal_handle, this]() { executeHoming(goal_handle); }}.detach();
        });

    const auto simMoveTask = SimTask::simMove;
    this->move_server_ = rclcpp_action::create_server<Move>(
        this, "~/move",
        [this, simMoveTask](auto, auto) { return handleGoal(simMoveTask); },
        [this, simMoveTask](const auto&) { return handleCancel(simMoveTask); },
        [this](const auto& goal_handle) {
          return std::thread{[goal_handle, this]() { executeMove(goal_handle); }}.detach();
        });

    const auto simGraspTask = SimTask::simGrasp;
    this->grasp_server_ = rclcpp_action::create_server<Grasp>(
        this, "~/grasp",
        [this, simGraspTask](auto, auto) { return handleGoal(simGraspTask); },
        [this, simGraspTask](const auto&) { return handleCancel(simGraspTask); },
        [this](const auto& goal_handle) {
          return std::thread{[goal_handle, this]() { executeGrasp(goal_handle); }}.detach();
        });

    const auto simGripperCommandTask = SimTask::simGripperCommand;
    this->gripper_command_server_ = rclcpp_action::create_server<GripperCommand>(
        this, "~/gripper_action",
        [this, simGripperCommandTask](auto, auto) { return handleGoal(simGripperCommandTask); },
        [this, simGripperCommandTask](const auto&) { return handleCancel(simGripperCommandTask); },
        [this](const auto& goal_handle) {
          return std::thread{[goal_handle, this]() { executeGripperCommand(goal_handle); }}
              .detach();
        });

    // Comunicazione con CoppeliaSim
    this->coppelia_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/franka/gripper/joint_commands", 10);
    this->feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/franka/joint_states", 10,
      std::bind(&GripperActionServerSim::feedback_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[INFO]: Gripper Action Server (Simulazione) avviato.");
  }

  rclcpp_action::CancelResponse GripperActionServerSim::handleCancel(SimTask task) {
    RCLCPP_INFO(this->get_logger(), "[INFO - SIMULAZIONE]: Ricevuta richiesta di annullamento per %s", getTaskName(task).c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  rclcpp_action::GoalResponse GripperActionServerSim::handleGoal(SimTask task) {
    RCLCPP_INFO(this->get_logger(), "[INFO - SIMULAZIONE]: Ricevuta richiesta per %s", getTaskName(task).c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void GripperActionServerSim::stopServiceCallback(const std::shared_ptr<Trigger::Response>& response) {
    RCLCPP_INFO(this->get_logger(), "[INFO - SIMULAZIONE]: Stop richiesto.");

    double current_pos_to_halt;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_pos_to_halt = current_width_;
    }

    sensor_msgs::msg::JointState stop_cmd;
    stop_cmd.name = {"fr3_finger_joint1", "fr3_finger_joint2"};
    stop_cmd.position = {current_pos_to_halt, current_pos_to_halt};
    coppelia_pub_->publish(stop_cmd);

    response->success = true;
    response->message = "Simulazione gripper fermata con successo.";

    RCLCPP_INFO(this->get_logger(), "[INFO - SIMULAZIONE]: Movimento interrotto alla posizione: %f", current_pos_to_halt);
  }

  void GripperActionServerSim::executeHoming(const std::shared_ptr<GoalHandleHoming>& goal_handle) {
    RCLCPP_INFO(this->get_logger(), "[INFO - SIMULAZIONE]: Avvio Homing...");
    const double target_pos = 0.04;
    
    sensor_msgs::msg::JointState cmd;
    cmd.name = {"fr3_finger_joint1", "fr3_finger_joint2"};
    cmd.position = {target_pos, target_pos};
    coppelia_pub_->publish(cmd);

    auto result = std::make_shared<Homing::Result>();
    if (waitForTarget(target_pos, goal_handle)) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "[INFO - SIMULAZIONE]: Homing completato.");
    } else {
        result->success = false;
        goal_handle->abort(result);
    }
  }

  void GripperActionServerSim::executeMove(const std::shared_ptr<GoalHandleMove>& goal_handle) {
    const auto goal = goal_handle->get_goal();
    const double target_pos = goal->width / 2.0;

    if( target_pos != current_width_ ){
      RCLCPP_INFO(this->get_logger(), "[INFO - SIMULAZIONE]: Muovo a larghezza %.3f", goal->width);

      sensor_msgs::msg::JointState cmd;
      cmd.name = {"fr3_finger_joint1", "fr3_finger_joint2"};
      cmd.position = {target_pos, target_pos};
      coppelia_pub_->publish(cmd);

      auto result = std::make_shared<Move::Result>();
      if (waitForTarget(target_pos, goal_handle)) {
          result->success = true;
          goal_handle->succeed(result);
      } else {
          result->success = false;
          goal_handle->abort(result);
      }
    }
  }

  void GripperActionServerSim::executeGrasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle) {
    const auto goal = goal_handle->get_goal();
    const double target_pos = goal->width / 2.0;

    RCLCPP_INFO(this->get_logger(), "[INFO - SIMULAZIONE]: Grasp a larghezza %.3f con forza %.1f N", goal->width, goal->force);

    sensor_msgs::msg::JointState cmd;
    cmd.name = {"fr3_finger_joint1", "fr3_finger_joint2"};
    cmd.position = {target_pos, target_pos};
    coppelia_pub_->publish(cmd);

    auto result = std::make_shared<Grasp::Result>();
    if (waitForTarget(target_pos, goal_handle)) {
        result->success = true;
        goal_handle->succeed(result);
    } else {
        result->success = false;
        goal_handle->abort(result);
    }
  }

  void GripperActionServerSim::executeGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle) {
    const auto goal = goal_handle->get_goal();
    const double target_pos =  2 * goal->command.position;

    RCLCPP_INFO(this->get_logger(), "[INFO - SIMULAZIONE]: Gripper Command posizione %.3f", goal->command.position);

    sensor_msgs::msg::JointState cmd;
    cmd.name = {"fr3_finger_joint1", "fr3_finger_joint2"};
    cmd.position = {target_pos, target_pos};
    coppelia_pub_->publish(cmd);

    auto result = std::make_shared<GripperCommand::Result>();
    if (waitForTarget(target_pos / 2, goal_handle)) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        result->effort = 0.0;
        result->position = current_width_;
        result->reached_goal = true;
        result->stalled = false;
        
        goal_handle->succeed(result);
      }
      
      RCLCPP_INFO(this->get_logger(), "[INFO - SIMULAZIONE]: Gripper Command completato con successo.");
    } 
    else {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        result->position = current_width_;
        result->reached_goal = false;
      }

      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "[INFO - SIMULAZIONE]: Gripper Command annullato.");
      } else {
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "[ERROR - SIMULAZIONE]: Gripper Command fallito per timeout o errore simulazione.");
      }
    }
  }

  void GripperActionServerSim::feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const std::string joint_name = "fr3_finger_joint1"; //
    auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
    if (it != msg->name.end()) {
      size_t index = std::distance(msg->name.begin(), it);
      current_width_ = msg->position[index];
    }
  }
}  // namespace single_arm

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<single_arm::GripperActionServerSim>());
    rclcpp::shutdown();
    return 0;
  }