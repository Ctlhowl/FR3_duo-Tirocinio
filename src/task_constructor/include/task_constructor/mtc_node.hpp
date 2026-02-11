#pragma once
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <thread>
#include <vector>
#include <string>

namespace task_constructor
{
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_task_node");
    namespace mtc = moveit::task_constructor;

    class MTCTaskNode
    {
    public:
        MTCTaskNode(const rclcpp::NodeOptions &options);
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
        void doTask();

    private:
        mtc::Task createTask();
        mtc::Task task_;
        rclcpp::Node::SharedPtr node_;

    };
} // namespace task_constructor
