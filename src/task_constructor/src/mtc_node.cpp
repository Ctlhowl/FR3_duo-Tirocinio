#include <task_constructor/mtc_node.hpp>

namespace task_constructor
{
  MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options): node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }{}

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
  {
    return node_->get_node_base_interface();
  }

  void MTCTaskNode::doTask()
  {
    rclcpp::sleep_for(std::chrono::seconds(2));
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
      RCLCPP_ERROR_STREAM(LOGGER, "[ERROR] Pianificazione del task fallita");
      return;
    }

    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR_STREAM(LOGGER, "Codice Errore: " << result.val);
      return;
    }
  }

  mtc::Task MTCTaskNode::createTask()
  {
    mtc::Task task;
    task.stages()->setName("FR3 Pick & Place");
    task.loadRobotModel(node_);

    const auto arm_group_name = "left_fr3_arm";
    const auto hand_group_name = "left_fr3_hand";
    const auto hand_frame = "left_fr3_hand_tcp";
    const auto object_name = "dynamic_cube";

    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

    // Pianificatore per i movimenti di base
    // PipelinePlanner (OMPL): Per movimenti liberi complessi
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    
    // CartesianPath: Per movimenti lineari (avvicinamento/sollevamento)
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    // JointInterpolation: Per movimenti semplici della pinza (Open/Close)
    auto gripper_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    // STAGE 1: Current State
    {
      auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
      task.add(std::move(stage_state_current));
    }

    // STAGE 2: Open Hand 
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open_hand", gripper_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      task.add(std::move(stage));
    }

    // STAGE 3: Move to Pick
    {
      auto stage = std::make_unique<mtc::stages::Connect>(
          "move_to_pick", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
      stage->setTimeout(5.0);
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage));
    }

    //mtc::Stage* attach_object_stage = nullptr;

    // STAGE 4: Pick Object
    {
      auto grasp = std::make_unique<mtc::SerialContainer>("pick_object");
      
      // Espone le proprietà alla task principale per facilitare la configurazione interna
      task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
      grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_collision (hand,object)");
        stage->allowCollisions(object_name,
                              task.getRobotModel()
                                  ->getJointModelGroup(hand_group_name)
                                  ->getLinkModelNamesWithCollisionGeometry(),
                              true);
        grasp->insert(std::move(stage));
      }

      // Approach Object
      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("approach_object", cartesian_planner);
        stage->properties().set("marker_ns", "approach_object");
        stage->properties().set("link", hand_frame);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.05, 0.1); // Avvicinamento tra 5cm e 10cm

        // Imposta la direzione lungo asse Z della mano
        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = hand_frame;
        direction.vector.z = 1.0; 
        stage->setDirection(direction);
        grasp->insert(std::move(stage));
      }
      
      // Generate Grasp Pose
      {
        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate_grasp_pose");
        stage->properties().set("marker_ns", "grasp_pose");
        stage->setPreGraspPose("open");
        stage->setObject(object_name);
        stage->setAngleDelta(M_PI / 2); // Scansiona ogni 15 gradi
        stage->setMonitoredStage(task.stages()->findChild("current"));
        stage->properties().configureInitFrom(mtc::Stage::PARENT);

        Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
        Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
        grasp_frame_transform.linear() = q.matrix();

        // Creiamo il wrapper ComputeIK per calcolare la cinematica inversa delle pose generate
        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp_pose_ik", std::move(stage));
        wrapper->setMaxIKSolutions(20);
        wrapper->setIKFrame(grasp_frame_transform, hand_frame);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        grasp->insert(std::move(wrapper));

      }

      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_collision (hand,object)");
        stage->allowCollisions(object_name,
                              task.getRobotModel()
                                  ->getJointModelGroup(hand_group_name)
                                  ->getLinkModelNamesWithCollisionGeometry(),
                              true);
        grasp->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close_hand", gripper_planner);
        stage->setGroup(hand_group_name);
        stage->setGoal("close");
        grasp->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach_object");
        stage->attachObject(object_name, hand_frame);
        grasp->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("lift_object", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.1, 0.3); // Sollevamento tra 10cm e 30cm
        stage->setIKFrame(hand_frame);
        stage->properties().set("marker_ns", "lift_object");


        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = "base";
        direction.vector.z = 1.0;
        stage->setDirection(direction);
        grasp->insert(std::move(stage));
      }

      task.add(std::move(grasp));
    }

    return task;
  }

} // namespace task_constructor

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_node = std::make_shared<task_constructor::MTCTaskNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_node](){
    executor.add_node(mtc_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_node->getNodeBaseInterface()); });
  
  rclcpp::sleep_for(std::chrono::seconds(2));
  mtc_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
