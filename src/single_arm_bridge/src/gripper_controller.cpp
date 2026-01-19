#pragma once
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "franka_msgs/action/grasp.hpp"

namespace single_arm_bridge {

class GraspController : public rclcpp::Node {
public:
    using Grasp = franka_msgs::action::Grasp;
    using GoalHandleGrasp = rclcpp_action::ClientGoalHandle<Grasp>;

    GraspController() : Node("grasp_controller") {
        // Init Grasp Action Client to communicate with GraspServer
        this->grasp_client_ = rclcpp_action::create_client<Grasp>(this, "franka_gripper/grasp");

        // Sottoscrizione a MoveIt2
        this->joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, 
            std::bind(&GraspController::joint_state_callback, this, std::placeholders::_1));
    }

private:
    void GraspController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Se il braccio raggiunge la posizione target pianificata da MoveIt2
        if (!grasp_executed_ && !msg->position.empty() && std::abs(msg->position[0] - target_trigger_pos_) < 0.05) {
            RCLCPP_INFO(this->get_logger(), "[INFO] Target reached. Launch Grasp on CoppeliaSim...");
            grasp_executed_ = true;
            
            send_grasp_goal(0.015, 100.0);
        }
    }

    // Function to send Grasp command
    void send_grasp_goal(double width, double force){
        auto goal_msg = Grasp::Goal();
        goal_msg.width = width;
        goal_msg.force = force;
        goal_msg.speed = 0.05;
        goal_msg.epsilon.inner = 0.005;
        goal_msg.epsilon.outer = 0.010;

        auto send_goal_options = rclcpp_action::Client<Grasp>::SendGoalOptions();
        send_goal_options.result_callback = [](const GoalHandleGrasp::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[INFO] Grasp completed successfully!");
            }
        };

        this->grasp_client_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    
    bool grasp_executed_ = false;
    double target_trigger_pos_ = 1.57;
};

} // namespace single_arm_bridge