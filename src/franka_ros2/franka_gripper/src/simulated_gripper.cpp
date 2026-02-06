#include "franka_gripper/simulated_gripper.hpp"

namespace franka_gripper {

SimulatedGripper::SimulatedGripper(rclcpp::Node* node) : node_(node) {
  node_->get_parameter("joint_names", joint_names_);

  // Topic verso CoppeliaSim
  cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("~/joint_commands", 10);
  
  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "~/joint_states_private", 10, 
      std::bind(&SimulatedGripper::jointStateCallback, this, std::placeholders::_1));
  
  current_state_.max_width = 0.08;
  current_effort_ = 0.0;
}

void SimulatedGripper::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  double width = 0.0;
  double effort_sum = 0.0;

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == joint_names_[0] || msg->name[i] ==  joint_names_[1]) {
      width += msg->position[i] / 2;
      if (msg->effort.size() > i) {
        effort_sum += std::abs(msg->effort[i]);
      }
    }
  }
  
  current_state_.width = width;
  current_effort_ = effort_sum;
  current_state_.is_grasped = (current_effort_ > k_effort_threshold);
}

bool SimulatedGripper::homing() { 
  return move(0.08, 0.1); 
}

bool SimulatedGripper::grasp(double width, double speed, double /*force*/, double /*in*/, double /*out*/) {
  return move(width, speed);
}

bool SimulatedGripper::stop() { 
  RCLCPP_INFO(node_->get_logger(), "Gripper: Richiesta di stop");
    
    // Segnala l'interruzione a eventuali cicli move/grasp in corso
    stop_requested_ = true;

    // Leggi la posizione attuale per "congelare" il gripper lì dove si trova
    franka::GripperState state = readOnce();
    
    // Invia immediatamente il comando di mantenere la posizione corrente
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->now();
    msg.name = joint_names_;
    msg.position.push_back(state.width / 2.0);
    msg.position.push_back(state.width / 2.0);
    cmd_pub_->publish(msg);

    return true;

}

bool SimulatedGripper::move(double width, double /*speed*/) {
  stop_requested_ = false;

  // Invia il comando
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();
  msg.name = joint_names_;
  msg.position.push_back(width / 2.0);
  msg.position.push_back(width / 2.0);
  cmd_pub_->publish(msg);

  // Parametri di controllo
  const double tolerance = 0.002;
  const auto timeout = std::chrono::seconds(3);
  auto start_time = std::chrono::steady_clock::now();

  // Loop di verifica
  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok()) {
    if (stop_requested_) {
      RCLCPP_WARN(node_->get_logger(), "Gripper: Movimento interrotto da comando STOP");
      return false;
    }

    franka::GripperState state = readOnce();
    
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Calcola l'errore assoluto rispetto al target
      double error = std::abs(state.width - width);
      
      if (error <= tolerance) {
        RCLCPP_INFO(node_->get_logger(), "Gripper: Target raggiunto (width: %f)", state.width);
        return true;
      }

      if (current_effort_ > k_effort_threshold) {
        RCLCPP_INFO(node_->get_logger(), "Gripper: Movimento interrotto da sforzo (Grasp rilevato)");
        return true;
      }

      if (std::chrono::steady_clock::now() - start_time > timeout) {
        RCLCPP_ERROR(node_->get_logger(), "Gripper: Timeout raggiunto prima di arrivare a target!");
        return false;
      }
    } 

    loop_rate.sleep();
  }
  
  return false;
}

franka::GripperState SimulatedGripper::readOnce() {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_;
}

} // namespace franka_gripper