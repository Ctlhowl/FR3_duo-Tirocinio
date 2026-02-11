#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

namespace task_constructor
{
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("object_spawner");
    
    class ObjectSpawner : public rclcpp::Node 
    {
    public:
        ObjectSpawner();
    private:
        void spawn_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr spawn_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr spawn_sub_;

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    };
} // namespace task_constructor