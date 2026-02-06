#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>

bool moveToState(moveit::planning_interface::MoveGroupInterface& group, const std::string& state_name, const rclcpp::Logger& logger) {
    RCLCPP_INFO(logger, "Pianificazione verso lo stato: %s", state_name.c_str());
    
    group.setNamedTarget(state_name);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(logger, "Esecuzione movimento...");
        group.execute(my_plan);
        return true;
    } else {
        RCLCPP_ERROR(logger, "Fallimento pianificazione per lo stato: %s", state_name.c_str());
        return false;
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>("start_pose_motion");
    auto const logger = rclcpp::get_logger("start_pose_motion");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Definizione dei gruppi
    using moveit::planning_interface::MoveGroupInterface;
    auto dual_arm = MoveGroupInterface(node, "dual_arm");


    dual_arm.setMaxVelocityScalingFactor(0.1);

    // Inizio sequenza
    moveToState(dual_arm, "ready", logger);

    RCLCPP_INFO(logger, "Sequenza completata con successo!");

    rclcpp::shutdown();
    return 0;
}