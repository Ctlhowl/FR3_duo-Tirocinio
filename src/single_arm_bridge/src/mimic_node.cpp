/*#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

class MimicNode : public rclcpp::Node {
public:
  MimicNode() : Node("mimic_node")
  {
    // Parametri
    in_joint_states_topic_ = this->declare_parameter<std::string>("in_joint_states_topic", "/joint_states");
    out_joint_cmd_topic_   = this->declare_parameter<std::string>("out_joint_cmd_topic", "/franka/left/coppelia_joint_commands");

    // Topic verso Coppelia per grasp "logico"
    attach_topic_ = this->declare_parameter<std::string>("attach_topic", "/coppelia/grasp/attach");
    detach_topic_ = this->declare_parameter<std::string>("detach_topic", "/coppelia/grasp/detach");

    // Nomi joint dita
    finger_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "finger_joint_names",
      std::vector<std::string>{"fr3_finger_joint1", "fr3_finger_joint2"}
    );

    // Se vuoi filtrare/riordinare SOLO i joint del braccio per Coppelia, mettili qui.
    // Se la tua Coppelia accetta tutto com'è, puoi lasciare vuoto.
    arm_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "arm_joint_names",
      std::vector<std::string>{"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"} 
    );

    // Soglie grasp
    // Per Franka hand tipico: aperto ~0.04 per dito, chiuso ~0.0.
    close_threshold_ = this->declare_parameter<double>("close_threshold", 0.008); // sotto -> consideriamo "chiuso"
    open_threshold_  = this->declare_parameter<double>("open_threshold",  0.020); // sopra -> consideriamo "aperto"

    // Debounce: quante callback consecutive devono soddisfare la condizione prima di attach/detach
    debounce_count_  = this->declare_parameter<int>("debounce_count", 3);

    // IO
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(in_joint_states_topic_, rclcpp::QoS(10), std::bind(&MimicNode::callback, this, std::placeholders::_1));

    pub_joint_cmd_ = this->create_publisher<sensor_msgs::msg::JointState>(out_joint_cmd_topic_, rclcpp::QoS(10));
    pub_attach_ = this->create_publisher<sensor_msgs::msg::JointState>(attach_topic_, rclcpp::QoS(10));
    pub_detach_ = this->create_publisher<sensor_msgs::msg::JointState>(detach_topic_, rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), " [mimic_node] Bridge avviato.");
    RCLCPP_INFO(this->get_logger(), "  in_joint_states_topic: %s", in_joint_states_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  out_joint_cmd_topic:   %s", out_joint_cmd_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  attach_topic:          %s", attach_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  detach_topic:          %s", detach_topic_.c_str());
  }

private:
  static std::optional<double> getJointPos(const sensor_msgs::msg::JointState& msg, const std::string& name)
  {
    auto iterator = std::find(msg.name.begin(), msg.name.end(), name);

    if (iterator == msg.name.end()){
        return std::nullopt;
    }

    const auto idx = static_cast<size_t>(std::distance(msg.name.begin(), iterator));
    if (idx >= msg.position.size()) {
        return std::nullopt;
    }

    return msg.position[idx];
  }

  void callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    sensor_msgs::msg::JointState out;

    if (!arm_joint_names_.empty()) {
      // Filtra e riordina solo i joint del braccio
      out.header = msg->header;
      out.name.reserve(arm_joint_names_.size());
      out.position.reserve(arm_joint_names_.size());

      for (const auto& joint_name : arm_joint_names_) {
        auto position = getJointPos(*msg, joint_name);

        if (!position.has_value()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[mimic_node] Joint '%s' non trovato in /joint_states", joint_name.c_str());
          continue;
        }

        out.name.push_back(joint_name);
        out.position.push_back(*position);
      }
    } else {
        out = *msg;
    }

    pub_joint_cmd_->publish(out);

    // Grasp
    // Calcola una "apertura media" dalle dita disponibili
    double sum = 0.0;
    int found = 0;
    for (const auto& finger_joint : finger_joint_names_) {
      auto position = getJointPos(*msg, finger_joint);
      if (position.has_value()) {
        sum += *position;
        found++;
      }
    }

    if (found == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "[mimic_node] Nessun finger joint trovato in /joint_states (controlla finger_joint_names).");
      return;
    }

    const double finger_avg = sum / static_cast<double>(found);

    if (!attached_) {
      if (finger_avg <= close_threshold_) {
        close_hits_++;
      } else {
        close_hits_ = 0;
      }

      if (close_hits_ >= debounce_count_) {
        std_msgs::msg::Bool b; b.data = true;
        pub_attach_->publish(b);
        attached_ = true;
        close_hits_ = 0;
        open_hits_ = 0;
        RCLCPP_INFO(this->get_logger(), "[mimic_node] Detected CLOSE (avg=%.5f) -> publish ATTACH", finger_avg);
      }
    } else {
      if (finger_avg >= open_threshold_) {
        open_hits_++;
      } else {
        open_hits_ = 0;
      }

      if (open_hits_ >= debounce_count_) {
        std_msgs::msg::Bool b; b.data = true;
        pub_detach_->publish(b);
        attached_ = false;
        close_hits_ = 0;
        open_hits_ = 0;
        RCLCPP_INFO(this->get_logger(), "[mimic_node] Detected OPEN (avg=%.5f) -> publish DETACH", finger_avg);
      }
    }
  }

  // Topics/parametri
  std::string in_joint_states_topic_;
  std::string out_joint_cmd_topic_;
  std::string attach_topic_;
  std::string detach_topic_;

  std::vector<std::string> finger_joint_names_;
  std::vector<std::string> arm_joint_names_;

  double close_threshold_{0.008};
  double open_threshold_{0.020};
  int debounce_count_{3};

  // Stati
  bool attached_{false};
  int close_hits_{0};
  int open_hits_{0};

  // ROS IO
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_attach_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_detach_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MimicNode>());
  rclcpp::shutdown();
  return 0;
}*/

#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class MimicNode : public rclcpp::Node {
public:
    MimicNode() : Node("mimic_node") {
        // Topic in entrata da MoveIt2
        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/franka/joint_states", 10, std::bind(&MimicNode::callback, this, std::placeholders::_1));

        // Topic in uscita verso CoppeliaSim
        pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/franka/left/coppelia_joint_commands", 10);

        RCLCPP_INFO(this->get_logger(), "[INFO]: Nodo Single Arm Bridge avviato.");
    }

private:
    void callback(const sensor_msgs::msg::JointState::SharedPtr msg) const {
        // Copia del messaggio ricevuto
        auto out_msg = std::make_shared<sensor_msgs::msg::JointState>(*msg);

        // Pubblicazione del messaggio modificato
        pub_->publish(*out_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MimicNode>());
    rclcpp::shutdown();
    return 0;
}
