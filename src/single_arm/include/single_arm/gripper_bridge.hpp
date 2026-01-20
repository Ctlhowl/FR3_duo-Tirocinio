#ifndef SINGLE_ARM__GRIPPER_BRIDGE_HPP_
#define SINGLE_ARM__GRIPPER_BRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/move.hpp"
#include "franka_msgs/action/homing.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include <control_msgs/action/gripper_command.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace single_arm {

class GripperBridge : public rclcpp::Node {
  public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GripperMove = franka_msgs::action::Move;
    using GripperGrasp = franka_msgs::action::Grasp;
    using GripperHomeing = franka_msgs::action::Homing;
    using Trigger = std_srvs::srv::Trigger;

    GripperBridge();

  private:
    double last_position_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_command_action_client_;
    rclcpp_action::Client<GripperMove>::SharedPtr gripper_move_action_client_;
    rclcpp_action::Client<GripperGrasp>::SharedPtr gripper_grasp_action_client_;
    rclcpp_action::Client<GripperHomeing>::SharedPtr gripper_homing_action_client_;
    rclcpp::Client<Trigger>::SharedPtr gripper_stop_client_;

    //Functions

    /**
    * Callback invocato ad ogni ricezione di un messaggio JointState da MoveIt
    * Cerca la posizione del fr3_finger_joint1 del gripper e invia il comando per impostare la posizione
    * @param msg Il messaggio JointState ricevuto
    */
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    /**
    * Invia un goal all'Action Server del gripper per impostare la posizione
    * @param position La posizione desiderata del gripper
    */
    void send_gripper_goal(double position);

    /**
    * Chiama il servizio di stop del gripper
    */
    void call_stop_service();

};

} // namespace single_arm

#endif  // SINGLE_ARM__GRIPPER_BRIDGE_HPP_