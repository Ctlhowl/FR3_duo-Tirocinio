#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_collision_object");

  moveit::planning_interface::PlanningSceneInterface psi;

  moveit_msgs::msg::CollisionObject obj;
  obj.id = "box1";
  obj.header.frame_id = "fr3_link0";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.05, 0.05, 0.05};

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 0.5;
  pose.position.y = 0.0;
  pose.position.z = 0.001;

  obj.primitives.push_back(primitive);
  obj.primitive_poses.push_back(pose);
  obj.operation = obj.ADD;

  psi.applyCollisionObject(obj);

  // Dai tempo al PlanningSceneMonitor
  rclcpp::sleep_for(std::chrono::milliseconds(300));

  rclcpp::shutdown();
  return 0;
}
