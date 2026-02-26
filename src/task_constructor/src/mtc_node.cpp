#include <task_constructor/mtc_node.hpp>

namespace task_constructor
{
  MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options) : node_{std::make_shared<rclcpp::Node>("mtc_node", options)} {}

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

    if (!task_.plan(10))
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
    task.stages()->setName("FR3 Dual Arm Pick");
    task.loadRobotModel(node_);

    const auto object_name = "dynamic_cube";

    // Planners
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    auto gripper_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    gripper_planner->setMaxVelocityScalingFactor(.001);
    gripper_planner->setMaxAccelerationScalingFactor(.001);

    {
      auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
      task.add(std::move(stage_state_current));
    }

    /****************************************************
     * LEFT ARM
     ***************************************************/
    {
      const auto arm = "left_fr3_arm";
      const auto hand = "left_fr3_hand";
      const auto tcp = "left_fr3_hand_tcp";

      auto left_robot = std::make_unique<mtc::SerialContainer>("left_arm_pick_sequence");
      left_robot->setProperty("group", arm);
      left_robot->setProperty("eef", hand);
      left_robot->setProperty("ik_frame", tcp);

      // Open Hand
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open_hand_left", gripper_planner);
        stage->setGroup(hand);
        stage->setGoal("open");
        left_robot->insert(std::move(stage));
      }

      // Connect to Pick Position
      {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "move_to_pick_left", mtc::stages::Connect::GroupPlannerVector{{arm, sampling_planner}});
        stage->setTimeout(5.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        left_robot->insert(std::move(stage));
      }

      // Approach & Grasp
      {
        auto grasp = std::make_unique<mtc::SerialContainer>("grasp_left");
        left_robot->properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
        grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        // Allow Collision
        {
          auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_collision_left");
          stage->allowCollisions(object_name,
                                 task.getRobotModel()->getJointModelGroup(hand)->getLinkModelNamesWithCollisionGeometry(),
                                 true);
          grasp->insert(std::move(stage));
        }

        // Approach
        {
          auto stage = std::make_unique<mtc::stages::MoveRelative>("approach_left", cartesian_planner);
          stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
          stage->setMarkerNS("approach_left");
          stage->setMinMaxDistance(0.05, 0.1);

          geometry_msgs::msg::Vector3Stamped direction;
          direction.header.frame_id = tcp;
          direction.vector.z = 1.0;
          stage->setDirection(direction);
          grasp->insert(std::move(stage));
        }

        // IK & Pose Generation
        {
          auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate_grasp_pose_left");
          stage->setPreGraspPose("open");
          stage->setObject(object_name);
          stage->setAngleDelta(M_PI / 4);
          stage->setMonitoredStage(task.stages()->findChild("current"));
          stage->properties().configureInitFrom(mtc::Stage::PARENT);

          Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
          Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
          grasp_frame_transform.linear() = q.matrix();

          auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp_pose_ik_left", std::move(stage));
          wrapper->setMaxIKSolutions(20);
          wrapper->setIKFrame(grasp_frame_transform, tcp);
          wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});
          wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
          grasp->insert(std::move(wrapper));
        }

        // Close & Attach
        {
          auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_collision (hand,object)");
          stage->allowCollisions(object_name,
                                task.getRobotModel()
                                    ->getJointModelGroup(hand)
                                    ->getLinkModelNamesWithCollisionGeometry(),
                                true);
          stage->attachObject(object_name, tcp);                                
          grasp->insert(std::move(stage));
        }

        {
          auto stage = std::make_unique<mtc::stages::MoveTo>("close_hand_left", gripper_planner);
          stage->setGroup(hand);
          stage->setGoal("close");
          grasp->insert(std::move(stage));
        }

        left_robot->insert(std::move(grasp));
      }

      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("lift_object", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.1, 0.3);
        stage->setIKFrame(tcp);
        stage->setMarkerNS("lift_object");

        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = "base";
        direction.vector.z = 1.0;
        stage->setDirection(direction);
        left_robot->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("move_left_to_handover", sampling_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        stage->setGroup(arm);
        stage->setGoal("left_handover_pose");
        left_robot->insert(std::move(stage));
      }

      task.add(std::move(left_robot));
    }

    /****************************************************
     * RIGHT ARM
     ***************************************************/
    {
      const auto arm = "right_fr3_arm";
      const auto hand = "right_fr3_hand";
      const auto tcp = "right_fr3_hand_tcp";

      auto right_robot = std::make_unique<mtc::SerialContainer>("right_arm_pick_sequence");
      right_robot->setProperty("group", arm);
      right_robot->setProperty("eef", hand);
      right_robot->setProperty("ik_frame", tcp);

      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open_hand_right", gripper_planner);
        stage->setGroup(hand);
        stage->setGoal("open");
        right_robot->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "connect_to_handover", mtc::stages::Connect::GroupPlannerVector{{arm, sampling_planner}});
        stage->setTimeout(5.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        right_robot->insert(std::move(stage));
      }

      // Container per Approccio + Posa Generata (Handover)
      {
        auto handover = std::make_unique<mtc::SerialContainer>("handover_approach_and_grasp");
        right_robot->properties().exposeTo(handover->properties(), {"eef", "group", "ik_frame"});
        handover->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        // Approccio
        {
          auto stage = std::make_unique<mtc::stages::MoveRelative>("approach_right", cartesian_planner);
          stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
          stage->setMarkerNS("approach_right");
          stage->setMinMaxDistance(0.05, 0.1);
          
          // Ci muoviamo in avanti lungo l'asse Z del TCP destro
          geometry_msgs::msg::Vector3Stamped direction;
          direction.header.frame_id = tcp;
          direction.vector.z = 1.0;
          stage->setDirection(direction);
          handover->insert(std::move(stage));
        }

        // Generazione della posa relativa all'altro braccio
        {
          auto stage = std::make_unique<mtc::stages::GeneratePose>("generate_handover_pose");
          stage->setMonitoredStage(task.stages()->findChild("left_arm_pick_sequence"));

          geometry_msgs::msg::PoseStamped target_pose;
          target_pose.header.frame_id = "left_fr3_hand_tcp";
          
          target_pose.pose.position.x = 0.0;
          target_pose.pose.position.y = 0.0;
          target_pose.pose.position.z = 0.0; 

          tf2::Quaternion q;
          q.setRPY(M_PI, 0, M_PI / 2);
          target_pose.pose.orientation.x = q.x();
          target_pose.pose.orientation.y = q.y();
          target_pose.pose.orientation.z = q.z();
          target_pose.pose.orientation.w = q.w();

          stage->setPose(target_pose);
          
          auto wrapper = std::make_unique<mtc::stages::ComputeIK>("compute_ik_handover_right", std::move(stage));
          wrapper->setMaxIKSolutions(8);
          wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
          wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
          wrapper->setIKFrame(tcp);

          handover->insert(std::move(wrapper));
        }

        right_robot->insert(std::move(handover));
      }

      //Trasferimento logico dell'oggetto nella scena
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("transfer_object");
        stage->allowCollisions(object_name, 
                              task.getRobotModel()
                                  ->getJointModelGroup(hand)
                                  ->getLinkModelNamesWithCollisionGeometry(),
                              true);
        
        stage->allowCollisions(task.getRobotModel()
                                  ->getJointModelGroup(hand)
                                  ->getLinkModelNamesWithCollisionGeometry(), 
                              task.getRobotModel()
                                  ->getJointModelGroup("left_fr3_hand")
                                  ->getLinkModelNamesWithCollisionGeometry(),
                              true);
        
        stage->detachObject(object_name, "left_fr3_hand_tcp");
        stage->attachObject(object_name, hand);
        right_robot->insert(std::move(stage));
      }

      // Chiusura pinza destra
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close_hand_right", gripper_planner);
        stage->setGroup(hand);
        stage->setGoal("close");
        right_robot->insert(std::move(stage));
      }

      // Apertura pinza sinistra
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open_hand_left", gripper_planner);
        stage->setGroup("left_fr3_hand");
        stage->setGoal("open");
        right_robot->insert(std::move(stage));
      }

      // 
      {
          auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat_left", cartesian_planner);
          stage->setGroup("left_fr3_arm");
          stage->setMarkerNS("retreat_left");
          stage->setMinMaxDistance(0.05, 0.1);
          
          geometry_msgs::msg::Vector3Stamped direction;
          direction.header.frame_id = "left_fr3_hand_tcp";
          direction.vector.z = -1.0;
          stage->setDirection(direction);
          right_robot->insert(std::move(stage));
        }

      // Fase di piazzamento
      {
        auto place = std::make_unique<mtc::SerialContainer>("place_sequence_right");
        right_robot->properties().exposeTo(place->properties(), {"group", "eef", "ik_frame"});
        place->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "eef", "ik_frame"});

        {
          auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_place_pose_2", sampling_planner);
          stage->setGroup(arm);
          stage->setIKFrame(tcp);
          stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});

          auto end_pose_vec = node_->get_parameter("end_pose").as_double_array();
          
          geometry_msgs::msg::PoseStamped target_pose;
          target_pose.header.frame_id = "base"; 
          
          target_pose.pose.position.x = end_pose_vec[0];
          target_pose.pose.position.y = 1.1;
          target_pose.pose.position.z = 0.6;

          Eigen::Quaterniond q_x(Eigen::AngleAxisd(-(M_PI / 2), Eigen::Vector3d::UnitX()));
          Eigen::Quaterniond q_z(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

          Eigen::Quaterniond q = q_x * q_z;
          q.normalize();

          target_pose.pose.orientation.x = q.x();
          target_pose.pose.orientation.y = q.y();
          target_pose.pose.orientation.z = q.z();
          target_pose.pose.orientation.w = q.w();

          stage->setGoal(target_pose);
          place->insert(std::move(stage));
        }

        {
          auto stage = std::make_unique<mtc::stages::MoveTo>("open_hand_release", gripper_planner);
          stage->setGroup(hand);
          stage->setGoal("open");
          place->insert(std::move(stage));
        }

        {
          auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach_object");
          stage->detachObject(object_name, hand);
          place->insert(std::move(stage));
        }

        {
          auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat_right", cartesian_planner);
          stage->setGroup(arm);
          stage->setMarkerNS("retreat_right");
          stage->setMinMaxDistance(0.05, 0.1);
          
          geometry_msgs::msg::Vector3Stamped direction;
          direction.header.frame_id = tcp;
          direction.vector.z = -1.0;
          stage->setDirection(direction);
          place->insert(std::move(stage));
        }

        {
          auto stage = std::make_unique<mtc::stages::MoveTo>("right_ready_pose", sampling_planner);
          stage->setGroup(arm);
          stage->setGoal("ready");
          place->insert(std::move(stage));
        }

        right_robot->insert(std::move(place));
      }

      task.add(std::move(right_robot));

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

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_node]()
                                                   {
    executor.add_node(mtc_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_node->getNodeBaseInterface()); });

  rclcpp::sleep_for(std::chrono::seconds(2));
  mtc_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}