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

        // Cerco giunto
        std::string master = "fr3_finger_joint1";

        auto iterator_master = std::find(out_msg->name.begin(), out_msg->name.end(), master);

        if (iterator_master != out_msg->name.end()) {
            size_t idx_master = std::distance(out_msg->name.begin(), iterator_master);

            // Impostiamo la posizione del giunto master pari al doppio della sua posizione originale 
            out_msg->position[idx_master] = 2 * out_msg->position[idx_master];
        }

        // Aggiorniamo il timestamp per sincronia
        out_msg->header.stamp = this->now();

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