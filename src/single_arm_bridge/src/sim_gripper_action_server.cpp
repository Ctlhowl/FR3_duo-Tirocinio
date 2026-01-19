#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "franka_msgs/action/grasp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class SimGripperActionServer : public rclcpp::Node {
public:
    using Grasp = franka_msgs::action::Grasp;
    using GoalHandleGrasp = rclcpp_action::ServerGoalHandle<Grasp>;

    SimGripperActionServer() : Node("sim_gripper_server") {
        // Publisher to CoppeliaSim
        this->sim_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/sim/gripper_width", 10);

        // Create Action Server that receives commands from Controller
        this->action_server_ = rclcpp_action::create_server<Grasp>(
            this, "franka_gripper/grasp",
            std::bind(&SimGripperActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SimGripperActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&SimGripperActionServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Grasp::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGrasp>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleGrasp> goal_handle) {
        std::thread{std::bind(&SimGripperActionServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleGrasp> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<Grasp::Result>();

        // Send the parameter to CoppeliaSim
        sensor_msgs::msg::JointState cmd;
        cmd.name.push_back("gripper_width");
        cmd.position.push_back(goal->width);
        sim_pub_->publish(cmd);

        RCLCPP_INFO(this->get_logger(), "Simulazione: Chiudo gripper a %f", goal->width);
        
        // Simula il tempo di chiusura fisica in Coppelia
        std::this_thread::sleep_for(std::chrono::seconds(2));

        result->success = true;
        goal_handle->succeed(result);
    }

    rclcpp_action::Server<Grasp>::SharedPtr action_server_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sim_pub_;
};