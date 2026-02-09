#pragma once
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include <thread>
#include <vector>
#include <string>

namespace fr3_mtc
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
        const std::string CUBE_ID = "cube_object";

        void setupPlanningScene(mtc::Task& task);
    };
} // namespace fr3_mtc
