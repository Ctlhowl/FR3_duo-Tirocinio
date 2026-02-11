#include <task_constructor/object_spawner.hpp>

namespace task_constructor
{
    ObjectSpawner::ObjectSpawner() : Node("object_spawner") {
        spawn_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cuboid/set_pose", 10);

        spawn_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/spawn_request", 10, 
            std::bind(&ObjectSpawner::spawn_callback, this, std::placeholders::_1));

        RCLCPP_INFO(LOGGER, "[INFO] Nodo Spawner Oggetto Coppelia avviato.");
    }

    void ObjectSpawner::spawn_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(LOGGER, "[INFO] Ricevuta richiesta spawn.");

        spawn_pub_->publish(*msg);
        
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header = msg->header;
        collision_object.id = "dynamic_cube";

        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        collision_object.primitives[0].dimensions = {0.04, 0.20, 0.04};

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