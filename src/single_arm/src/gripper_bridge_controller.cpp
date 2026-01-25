#include "single_arm/gripper_bridge_controller.hpp"

namespace single_arm {

controller_interface::InterfaceConfiguration GripperBridgeController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration GripperBridgeController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE};
}

CallbackReturn GripperBridgeController::on_init() {
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperBridgeController::on_configure(const rclcpp_lifecycle::State&) {
  auto node = get_node();

  // Inizializzazione Server per MoveIt
  moveit_action_server_ = rclcpp_action::create_server<GripperCommand>(
    node, "gripper_action",
    std::bind(&GripperBridgeController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GripperBridgeController::handle_cancel, this, std::placeholders::_1),
    std::bind(&GripperBridgeController::handle_accepted, this, std::placeholders::_1));

  // Inizializzazione Client verso l'esterno (Hardware o Coppelia)
  gripper_grasp_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(node, "/franka_gripper/grasp");
  gripper_move_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(node, "/franka_gripper/move");

  return CallbackReturn::SUCCESS;
}

void GripperBridgeController::handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle) {
  const auto goal = goal_handle->get_goal();
  double pos = goal->command.position;

  RCLCPP_INFO(get_node()->get_logger(), "[INFO]: Ricevuto comando MoveIt: posizione %.3f", pos);

  // LOGICA BRIDGE: Se la posizione richiesta è minore della soglia di chiusura, è un GRASP
  if (pos < closing_threshold_) {
    call_hardware_grasp(goal_handle);
  } else {
    call_hardware_move(goal_handle, pos);
  }
}

void GripperBridgeController::call_hardware_grasp(const std::shared_ptr<GoalHandleGripper> goal_handle) {
  auto internal_goal = franka_msgs::action::Grasp::Goal();
  internal_goal.width = 0.00; // Chiudi tutto finché non senti una resistenza
  internal_goal.speed = 0.1;
  internal_goal.force = 40.0;
  internal_goal.epsilon.inner = 0.005;
  internal_goal.epsilon.outer = 0.005;

  auto options = rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions();

  options.result_callback = [goal_handle](const auto& result) {
    auto res = std::make_shared<GripperCommand::Result>();
    res->reached_goal = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    goal_handle->succeed(res);
  };

  gripper_grasp_client_->async_send_goal(internal_goal, options);
}

void GripperBridgeController::call_hardware_move(const std::shared_ptr<GoalHandleGripper> goal_handle, double position) {
  auto internal_goal = franka_msgs::action::Move::Goal();
  internal_goal.width = position;
  internal_goal.speed = 0.1;

  auto options = rclcpp_action::Client<franka_msgs::action::Move>::SendGoalOptions();
  options.result_callback = [goal_handle](const auto& result) {
    auto res = std::make_shared<GripperCommand::Result>();
    res->reached_goal = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    goal_handle->succeed(res);
  };

  gripper_move_client_->async_send_goal(internal_goal, options);
}

// Boilerplate per lifecycle
CallbackReturn GripperBridgeController::on_activate(const rclcpp_lifecycle::State&) { return CallbackReturn::SUCCESS; }
CallbackReturn GripperBridgeController::on_deactivate(const rclcpp_lifecycle::State&) { return CallbackReturn::SUCCESS; }
controller_interface::return_type GripperBridgeController::update(const rclcpp::Time&, const rclcpp::Duration&) { return controller_interface::return_type::OK; }
rclcpp_action::GoalResponse GripperBridgeController::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const GripperCommand::Goal>) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
rclcpp_action::CancelResponse GripperBridgeController::handle_cancel(const std::shared_ptr<GoalHandleGripper>) { return rclcpp_action::CancelResponse::ACCEPT; }

} // namespace single_arm

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(single_arm::GripperBridgeController, controller_interface::ControllerInterface)