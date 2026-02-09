#include <fr3_mtc/mtc_task_node.hpp>

namespace fr3_mtc
{
  MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options): node_{ std::make_shared<rclcpp::Node>("mtc_task_node", options) }{}

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
  {
    return node_->get_node_base_interface();
  }

  void MTCTaskNode::setupPlanningScene(mtc::Task& task)
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = CUBE_ID;
    object.header.frame_id = "base";

    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = {0.04, 0.20, 0.04};

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5;
    pose.position.y = 0.0;
    pose.position.z = 0.02;

    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 1.2;

    object.pose = pose;
    object.operation = object.ADD;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);

    auto add_cube = std::make_unique<mtc::stages::ModifyPlanningScene>("add_cube_to_scene");
    add_cube->addObject(object);
    task.add(std::move(add_cube));
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

    const std::vector<std::string> right_fingers = {
        "right_fr3_leftfinger", "right_fr3_rightfinger"};
    const std::vector<std::string> left_fingers = {
        "left_fr3_leftfinger", "left_fr3_rightfinger"};

    setupPlanningScene(task);
    
    // --- PREPARAZIONE ---
    {
      auto start_right_arm = std::make_unique<mtc::stages::MoveTo>("start_right_arm", pipeline_planner);
      start_right_arm->setGroup("right_fr3_arm");
      start_right_arm->setGoal("ready");
      task.add(std::move(start_right_arm));

      auto start_right_hand = std::make_unique<mtc::stages::MoveTo>("start_right_hand", pipeline_planner);
      start_right_hand->setGroup("right_fr3_hand");
      start_right_hand->setGoal("open");
      task.add(std::move(start_right_hand));

      auto start_left_arm = std::make_unique<mtc::stages::MoveTo>("start_left_arm", pipeline_planner);
      start_left_arm->setGroup("left_fr3_arm");
      start_left_arm->setGoal("ready");
      task.add(std::move(start_left_arm));

      auto start_left_hand = std::make_unique<mtc::stages::MoveTo>("start_left_hand", pipeline_planner);
      start_left_hand->setGroup("left_fr3_hand");
      start_left_hand->setGoal("open");
      task.add(std::move(start_left_hand));
    }

    // --- PICK DESTRA ---
    {
      auto right_approach = std::make_unique<mtc::stages::MoveTo>("right_approach", pipeline_planner);
      right_approach->setGroup("right_fr3_arm");
      right_approach->setGoal("right_pick_approach");
      task.add(std::move(right_approach));

      auto allow_cube_grasp = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_cube_right_hand");
      allow_cube_grasp->allowCollisions(CUBE_ID, right_hand_links, true);
      task.add(std::move(allow_cube_grasp));

      auto right_pre_grasp = std::make_unique<mtc::stages::MoveTo>("right_pre_grasp", pipeline_planner);
      right_pre_grasp->setGroup("right_fr3_arm");
      right_pre_grasp->setGoal("right_pick_cube");
      task.add(std::move(right_pre_grasp));

      auto right_grasp = std::make_unique<mtc::stages::MoveTo>("right_grasp", pipeline_planner);
      right_grasp->setTimeout(5.0);
      right_grasp->setGroup("right_fr3_hand");
      right_grasp->setGoal("close");
      task.add(std::move(right_grasp));

      auto attach_right = std::make_unique<mtc::stages::ModifyPlanningScene>("attach_cube_right");
      attach_right->attachObject(CUBE_ID, "right_fr3_hand");
      attach_right->allowCollisions(CUBE_ID, right_hand_links, true);
      task.add(std::move(attach_right));
    }

    // --- MOVIMENTO DI EXCHANGE ---
    {
      auto allow_finger_finger = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_fingers");
      allow_finger_finger->allowCollisions(right_fingers, left_fingers, true);
      task.add(std::move(allow_finger_finger));

      auto move_to_exchange = std::make_unique<mtc::stages::MoveTo>("move_to_exchange", pipeline_planner);
      move_to_exchange->setGroup("right_fr3_arm");
      move_to_exchange->setGoal("right_to_left");
      task.add(std::move(move_to_exchange));
    }

    // --- PICK SINISTRA ---
    {
      auto left_approach = std::make_unique<mtc::stages::MoveTo>("left_approach", pipeline_planner);
      left_approach->setGroup("left_fr3_arm");
      left_approach->setGoal("left_pick_approach");
      task.add(std::move(left_approach));

      auto allow_exchange_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_exchange_collision");
      allow_exchange_collision->allowCollisions(CUBE_ID, right_hand_links, true);
      allow_exchange_collision->allowCollisions(CUBE_ID, left_hand_links, true);
      allow_exchange_collision->allowCollisions(right_hand_links, left_hand_links, true);
      task.add(std::move(allow_exchange_collision));

      auto left_pre_grasp = std::make_unique<mtc::stages::MoveTo>("left_pre_grasp", pipeline_planner);
      left_pre_grasp->setGroup("left_fr3_arm");
      left_pre_grasp->setGoal("left_pick_cube");
      task.add(std::move(left_pre_grasp));

      auto left_grasp = std::make_unique<mtc::stages::MoveTo>("left_grasp", pipeline_planner);
      left_grasp->setTimeout(5.0);
      left_grasp->setGroup("left_fr3_hand");
      left_grasp->setGoal("close");
      task.add(std::move(left_grasp));

      auto detach_right= std::make_unique<mtc::stages::ModifyPlanningScene>("detach_cube_right");
      detach_right->detachObject(CUBE_ID, "right_fr3_hand");
      task.add(std::move(detach_right));

      auto attach_left = std::make_unique<mtc::stages::ModifyPlanningScene>("attach_cube_left");
      attach_left->attachObject(CUBE_ID, "left_fr3_hand");
      attach_left->allowCollisions(CUBE_ID, left_hand_links, true);
      task.add(std::move(attach_left));

      auto open_right_hand = std::make_unique<mtc::stages::MoveTo>("open_right_hand_exchange", pipeline_planner);
      open_right_hand->setGroup("right_fr3_hand");
      open_right_hand->setGoal("open");
      task.add(std::move(open_right_hand));
    }

    // --- ALLONTANAMENTO E POSA FINALE ---
    {
      auto dual_move = std::make_unique<mtc::stages::MoveTo>("dual_arm_move", pipeline_planner);
      dual_move->setGroup("dual_arm");
      dual_move->setGoal("dual_place_safe");
      task.add(std::move(dual_move));

      auto right_ready = std::make_unique<mtc::stages::MoveTo>("right_back_to_ready", pipeline_planner);
      right_ready->setGroup("right_fr3_arm");
      right_ready->setGoal("ready");
      task.add(std::move(right_ready));
      
      auto restore_collisions = std::make_unique<mtc::stages::ModifyPlanningScene>("restore_collisions");
      restore_collisions->allowCollisions(CUBE_ID, right_hand_links, false);
      restore_collisions->allowCollisions(right_fingers, left_fingers, false);
      task.add(std::move(restore_collisions));

      auto left_place = std::make_unique<mtc::stages::MoveTo>("left_place", pipeline_planner);
      left_place->setGroup("left_fr3_arm");
      left_place->setGoal("left_place_cube");
      task.add(std::move(left_place));

      auto left_open = std::make_unique<mtc::stages::MoveTo>("left_open", pipeline_planner);
      left_open->setGroup("left_fr3_hand");
      left_open->setGoal("open");
      task.add(std::move(left_open));

      auto detach_left = std::make_unique<mtc::stages::ModifyPlanningScene>("detach_left");
      detach_left->detachObject(CUBE_ID, "left_fr3_hand");
      task.add(std::move(detach_left));
    }

    return task;
  }

} // namespace fr3_mtc

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<fr3_mtc::MTCTaskNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]()
                                                   {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface()); });
  
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  // Run task once
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
