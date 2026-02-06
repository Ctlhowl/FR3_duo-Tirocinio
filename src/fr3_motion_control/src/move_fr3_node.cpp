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
    auto const node = std::make_shared<rclcpp::Node>("move_fr3_node");
    auto const logger = rclcpp::get_logger("move_fr3_node");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Definizione dei gruppi
    using moveit::planning_interface::MoveGroupInterface;
    auto arm_right = MoveGroupInterface(node, "right_fr3_arm");
    auto arm_left = MoveGroupInterface(node, "left_fr3_arm");
    auto dual_arm = MoveGroupInterface(node, "dual_arm");
    auto hand_right = MoveGroupInterface(node, "right_fr3_hand");
    auto hand_left = MoveGroupInterface(node, "left_fr3_hand");

    arm_right.setMaxVelocityScalingFactor(0.1);
    arm_left.setMaxVelocityScalingFactor(0.1);
    dual_arm.setMaxVelocityScalingFactor(0.1);
    hand_right.setMaxVelocityScalingFactor(0.1);
    hand_left.setMaxVelocityScalingFactor(0.1);

    // Inizio sequenza
    if(moveToState(arm_right, "right_pick_approach", logger)) {
        moveToState(arm_right, "right_pick_cube", logger);
        moveToState(hand_right, "close", logger);
    }

    moveToState(arm_right, "right_to_left", logger);

    if(moveToState(arm_left, "left_pick_approach", logger)) {
        moveToState(arm_left, "left_pick_cube", logger);
        moveToState(hand_left, "close", logger);
        moveToState(hand_right, "open", logger);
    }

    RCLCPP_INFO(logger, "Esecuzione movimento coordinato dual-arm...");
    moveToState(dual_arm, "dual_place_safe", logger);

    moveToState(arm_left, "left_place_cube", logger);
    moveToState(hand_left, "open", logger);

    RCLCPP_INFO(logger, "Sequenza completata con successo!");

    rclcpp::shutdown();
    return 0;
}