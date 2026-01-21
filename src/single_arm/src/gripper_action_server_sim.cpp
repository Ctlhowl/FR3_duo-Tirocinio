#include <single_arm/gripper_action_server_sim.hpp>

namespace single_arm {

GripperActionServerSim::GripperActionServerSim() : Node("franka_gripper_node") {
    // Inizializzazione stati e nomi giunti
    joint_names_ = {"fr3_finger_joint1", "fr3_finger_joint2"};
    current_gripper_state_.max_width = max_gripper_width_;
    current_gripper_state_.width = max_gripper_width_;

    // Sottoscrizione allo stato reale da Coppelia 
    coppelia_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/sim/franka/gripper/joint_states", 10, 
        std::bind(&GripperActionServerSim::onCoppeliaJointState, this, std::placeholders::_1));

    // Publisher verso Coppelia
    coppelia_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/sim/franka/gripper/joint_commands", 10);
    
    // Publisher ROS standard per lo stato del gripper
    joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&GripperActionServerSim::publishGripperState, this));

    // Services
     this->stop_service_ = create_service<Trigger>(
        "~/stop", 
        [this](std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response) { 
          return stopServiceCallback(std::move(response)); 
        });

    // Action Servers
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
          return std::thread{[goal_handle, this]() { onExecuteGripperCommand(goal_handle); }}.detach();
        });
    RCLCPP_INFO(this->get_logger(), "[INFO - SIMULAZIONE]: Gripper Action Server pronto.");
}

std::string GripperActionServerSim::getTaskName(SimTask task) {
    switch (task) {
        case SimTask::simHoming: return "Homing";
        case SimTask::simMove: return "Move";
        case SimTask::simGrasp: return "Grasp";
        case SimTask::simGripperCommand: return "GripperCommand";
        default: return "Unknown";
    }
}

void GripperActionServerSim::onCoppeliaJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(gripper_sim_state_mutex_);
    if (msg->position.size() >= 2) {
        current_gripper_state_.width = msg->position[0] + msg->position[1];
    }
}

void GripperActionServerSim::sendToCoppelia(double target_width) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position.push_back(target_width / 2.0);
    msg.position.push_back(target_width / 2.0);
    coppelia_pub_->publish(msg);
}

// --- Implementazione Action Logic ---
void GripperActionServerSim::executeHoming(const std::shared_ptr<GoalHandleHoming>& goal_handle) {
    auto cmd = [this, goal_handle]() {
        sendToCoppelia(max_gripper_width_);
        return waitForTarget(max_gripper_width_, goal_handle);
    };

    executeCommand(goal_handle, SimTask::simHoming, cmd);
}

void GripperActionServerSim::executeMove(const std::shared_ptr<GoalHandleMove>& goal_handle) {
    double target = goal_handle->get_goal()->width;

    auto cmd = [this, target, goal_handle]() {
        sendToCoppelia(target);
        return waitForTarget(target, goal_handle);
    };
    
    executeCommand(goal_handle, SimTask::simMove, cmd);
}

void GripperActionServerSim::executeGrasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle) {
    double target = goal_handle->get_goal()->width;
    
    auto cmd = [this, target, goal_handle]() {
        sendToCoppelia(target);
        // In simulazione considero il Grasp come un Move; in un sistema reale
        // si fermerebbe al contatto misurando l'effort.
        return waitForTarget(target, goal_handle);
    };
    
    executeCommand(goal_handle, SimTask::simGrasp, cmd);
}

void GripperActionServerSim::onExecuteGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle) {
    const auto goal = goal_handle->get_goal();

    // Per GripperCommand, la position nel goal è quella di un singolo dito
    double targetWidth = 2.0 * goal->command.position;

    sendToCoppelia(targetWidth);

    auto result = std::make_shared<GripperCommand::Result>();
    bool success = waitForTarget(targetWidth, goal_handle);

    {
        std::lock_guard<std::mutex> lock(gripper_sim_state_mutex_);
        result->position = current_gripper_state_.width;
        result->reached_goal = success;
    }

    if (success) goal_handle->succeed(result);
    else goal_handle->abort(result);
}

// --- Helper & Callbacks ---
void GripperActionServerSim::publishGripperState() {
    std::lock_guard<std::mutex> lock(gripper_sim_state_mutex_);
    sensor_msgs::msg::JointState joint_states;
    joint_states.header.stamp = this->now();
    joint_states.name.push_back(this->joint_names_[0]);
    joint_states.name.push_back(this->joint_names_[1]);
    joint_states.position.push_back(current_gripper_state_.width / 2);
    joint_states.position.push_back(current_gripper_state_.width / 2);
    joint_states.velocity.push_back(0.0);
    joint_states.velocity.push_back(0.0);
    joint_states.effort.push_back(0.0);
    joint_states.effort.push_back(0.0);
    //joint_states_publisher_->publish(joint_states);
}

void GripperActionServerSim::stopServiceCallback(const std::shared_ptr<Trigger::Response>& response) {
    std::lock_guard<std::mutex> lock(gripper_sim_state_mutex_);
    sendToCoppelia(current_gripper_state_.width);
    response->success = true;
    response->message = "Gripper fermato in posizione corrente.";
}

rclcpp_action::CancelResponse GripperActionServerSim::handleCancel(SimTask task) {
    RCLCPP_INFO(this->get_logger(), "Cancellazione %s", getTaskName(task).c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::GoalResponse GripperActionServerSim::handleGoal(SimTask task) {
    RCLCPP_INFO(this->get_logger(), "Ricevuto goal %s", getTaskName(task).c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

} // namespace single_arm

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<single_arm::GripperActionServerSim>());
    rclcpp::shutdown();
    return 0;
}