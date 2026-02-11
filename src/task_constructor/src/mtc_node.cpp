#include <task_constructor/mtc_node.hpp>

namespace task_constructor
{
  MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options): node_{ std::make_shared<rclcpp::Node>("mtc_task_node", options) }{}

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
  {
    return node_->get_node_base_interface();
  }

  void MTCTaskNode::doTask()
  {
    task_ = createTask();

    try
    {
      task_.init();
    }
    catch (mtc::InitStageException &e)
    {
      RCLCPP_ERROR_STREAM(LOGGER, e);
      return;
    }

    if (!task_.plan(40))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
      return;
    }

    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
      return;
    }
  }

  mtc::Task MTCTaskNode::createTask()
  {
    mtc::Task task;
    task.stages()->setName("Dual Arm Exchange");
    task.loadRobotModel(node_);

    auto pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    
    task.add(std::make_unique<mtc::stages::CurrentState>("current_state"));

    const auto right_hand_links =
        task.getRobotModel()
            ->getJointModelGroup("right_fr3_hand")
            ->getLinkModelNamesWithCollisionGeometry();

    const auto left_hand_links =
        task.getRobotModel()
            ->getJointModelGroup("left_fr3_hand")
            ->getLinkModelNamesWithCollisionGeometry();

    const std::vector<std::string> right_fingers = {"right_fr3_leftfinger", "right_fr3_rightfinger"};
    const std::vector<std::string> left_fingers = {"left_fr3_leftfinger", "left_fr3_rightfinger"};
    
    return task;
  }

} // namespace task_constructor

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<task_constructor::MTCTaskNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]()
                                                   {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface()); });
  
  // Run task once
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
