#include <memory>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <single_arm/gripper_bridge.hpp>

namespace single_arm {
  GripperBridge::GripperBridge() : Node("gripper_bridge") {
    last_position_ = 0.4;

    // Sottoscrizione al Topic per le informazioni dei giunti
    this->subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/franka/joint_states", 
      10,
      std::bind(&GripperBridge::joint_state_callback, this, std::placeholders::_1)
    );

    // Creazione del client per l'Action Server del gripper
    this->gripper_command_action_client_ = rclcpp_action::create_client<GripperCommand>(this, "/franka_gripper_node/gripper_action");
    this->gripper_move_action_client_ = rclcpp_action::create_client<GripperMove>(this, "/franka_gripper_node/move");
    this->gripper_grasp_action_client_ = rclcpp_action::create_client<GripperGrasp>(this, "/franka_gripper_node/grasp");
    this->gripper_homing_action_client_ = rclcpp_action::create_client<GripperHomeing>(this, "/franka_gripper_node/homing");
    this->gripper_stop_client_ = this->create_client<Trigger>("/franka_gripper_node/stop");

    RCLCPP_INFO(this->get_logger(), "[INFO] Gripper Bridge Node avviato.");
  }

  void GripperBridge::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    const std::string finger_joint = "fr3_finger_joint1"; 
    auto it = std::find(msg->name.begin(), msg->name.end(), finger_joint);
    
    if (it != msg->name.end()) {
      size_t index = std::distance(msg->name.begin(), it);
      double position = msg->position[index];
      send_gripper_goal(position);
    }
  }
  
  void GripperBridge::send_gripper_goal(double target_position) {
    if (last_position_ == target_position) {
      return;
    }

    /*

    if (this->gripper_command_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        auto goal_msg = GripperCommand::Goal();
        goal_msg.command.position = target_position;
        goal_msg.command.max_effort = 15;

        RCLCPP_INFO(this->get_logger(), "[BRIDGE] Eseguo Gripper Command");
        this->gripper_command_action_client_->async_send_goal(goal_msg);
        last_position_ = target_position;
        return;
      }

    */
   
    // Logica per HOMING
    if (target_position >= 0.039) {
      if (this->gripper_homing_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        auto goal_msg = GripperHomeing::Goal();
        RCLCPP_INFO(this->get_logger(), "[BRIDGE] Eseguo HOMING");
        this->gripper_homing_action_client_->async_send_goal(goal_msg);
        last_position_ = target_position;
        return;
      }
    }

    // Logica per GRASP (Se la posizione è minore dell'ultima posizione inviata)
    if (target_position < last_position_) {
      if (this->gripper_grasp_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        auto goal_msg = GripperGrasp::Goal();
        goal_msg.width = target_position * 2.0; // Larghezza totale
        goal_msg.speed = 0.1;
        goal_msg.force = 50.0;
        goal_msg.epsilon.inner = 0.005;
        goal_msg.epsilon.outer = 0.005;

        RCLCPP_INFO(this->get_logger(), "[BRIDGE] Eseguo GRASP (Chiusura)");
        this->gripper_grasp_action_client_->async_send_goal(goal_msg);
        last_position_ = target_position;
        return;
      }
    }
    

    // Logica per MOVE
    if (this->gripper_move_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      auto goal_msg = GripperMove::Goal();
      goal_msg.width = target_position * 2.0;
      goal_msg.speed = 0.05;

      RCLCPP_INFO(this->get_logger(), "[BRIDGE] Eseguo MOVE");
      this->gripper_move_action_client_->async_send_goal(goal_msg);
      last_position_ = target_position;
    }
        
  }

  void GripperBridge::call_stop_service() {
    if (!this->gripper_stop_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "[ERRORE]: Servizio STOP non disponibile!");
      return;
    }

    auto request = std::make_shared<Trigger::Request>();
    
    this->gripper_stop_client_->async_send_request(request, 
      [this](rclcpp::Client<Trigger>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "[BRIDGE] Stop eseguito: %s", response->message.c_str());
        }
      });
  }
  
} // namespace single_arm

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<single_arm::GripperBridge>());
  rclcpp::shutdown();
  return 0;
}
