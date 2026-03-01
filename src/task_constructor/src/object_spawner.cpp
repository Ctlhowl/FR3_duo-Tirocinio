#include <task_constructor/object_spawner.hpp>

namespace task_constructor
{
    ObjectSpawner::ObjectSpawner() : Node("object_spawner") {
        this->declare_parameter("start_pose", std::vector<double>{0.5, -0.5, 1});

        spawn_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cuboid/set_pose", 10);

        spawn_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/spawn_request", 10, 
            std::bind(&ObjectSpawner::spawn_callback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), 
            std::bind(&ObjectSpawner::spawn_from_params, this));
        
            RCLCPP_INFO(LOGGER, "[INFO] Nodo Spawner Oggetto avviato.");
    }

    void ObjectSpawner::spawn_from_params() {
        if (spawn_pub_->get_subscription_count() == 0) {
            RCLCPP_WARN(this->get_logger(), "[WARNING] Simulatore non ancora connesso a /cuboid/set_pose. Riprovo...");
            return;
        }

        timer_->cancel();

        auto start_pose_vec = this->get_parameter("start_pose").as_double_array();

        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "base";
        msg.header.stamp = this->now();
        msg.pose.position.x = start_pose_vec[0];
        msg.pose.position.y = start_pose_vec[1];;
        msg.pose.position.z = 0.02;
        msg.pose.orientation.z = start_pose_vec[2];;
        msg.pose.orientation.w = 1.0;

        this->spawn_callback(std::make_shared<geometry_msgs::msg::PoseStamped>(msg));
    }

    void ObjectSpawner::spawn_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(LOGGER, "[INFO] Ricevuta richiesta spawn.");

        spawn_pub_->publish(*msg);
        
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header = msg->header;
        collision_object.id = "dynamic_cube";

        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        collision_object.primitives[0].dimensions = {0.05, 0.05, 0.05};

        collision_object.pose = msg->pose;

        collision_object.operation = collision_object.ADD;

        planning_scene_interface_.applyCollisionObject(collision_object);

        RCLCPP_INFO(LOGGER, "[INFO] Oggetto spawnato e aggiunto alla scena.");
    }

} // namespace task_constructor

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<task_constructor::ObjectSpawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}