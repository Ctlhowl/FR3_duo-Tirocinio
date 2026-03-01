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

    RCLCPP_INFO(LOGGER, "[INFO] Task completato. Rimozione del cubo dalla scena per il prossimo run...");
    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<std::string> objects_to_remove = { "dynamic_cube" };
    psi.removeCollisionObjects(objects_to_remove);
    
    RCLCPP_INFO(LOGGER, "[INFO] Cubo rimosso con successo dalla Planning Scene.");
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
        auto stage = std::make_unique<mtc::stages::MoveTo>("open_hand", gripper_planner);
        stage->setGroup(hand);
        stage->setGoal("open");
        left_robot->insert(std::move(stage));
      }

      // Connect to Pick Position
      {
        auto stage = std::make_unique<mtc::stages::Connect>("move_to_pick_pose", mtc::stages::Connect::GroupPlannerVector{{arm, sampling_planner}});
        stage->setTimeout(15.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        left_robot->insert(std::move(stage));
      }

      // Approach & Grasp
      {
        auto grasp = std::make_unique<mtc::SerialContainer>("grasp");
        left_robot->properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
        grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        // Allow Collision
        {
          auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_collision (object, hand)");
          stage->allowCollisions(object_name,
                                 task.getRobotModel()->getJointModelGroup(hand)->getLinkModelNamesWithCollisionGeometry(),
                                 true);
          grasp->insert(std::move(stage));
        }

        // Approach
        {
          auto stage = std::make_unique<mtc::stages::MoveRelative>("grasp_approach", cartesian_planner);
          stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
          stage->setMarkerNS("grasp_approach");
          stage->setMinMaxDistance(0.01, 0.1);

          geometry_msgs::msg::Vector3Stamped direction;
          direction.header.frame_id = tcp;
          direction.vector.z = 1.0;
          stage->setDirection(direction);
          grasp->insert(std::move(stage));
        }

        // IK & Pose Generation
        {
          auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate_grasp_pose");
          stage->setPreGraspPose("open");
          stage->setObject(object_name);
          stage->setAngleDelta(M_PI / 2);
          stage->setMonitoredStage(task.stages()->findChild("current"));
          stage->properties().configureInitFrom(mtc::Stage::PARENT);

          Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
          Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
          grasp_frame_transform.linear() = q.matrix();

          auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp_pose_ik", std::move(stage));
          wrapper->setMaxIKSolutions(1);
          wrapper->setIKFrame(grasp_frame_transform, tcp);
          wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});
          wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
          grasp->insert(std::move(wrapper));
        }

        // Close & Attach
        {
          auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach (object, hand)");
          stage->allowCollisions(object_name,
                                  task.getRobotModel()->getJointModelGroup(hand)->getLinkModelNamesWithCollisionGeometry(),
                                  true);

          stage->attachObject(object_name, tcp);
                                          
          grasp->insert(std::move(stage));
        }

        {
          auto stage = std::make_unique<mtc::stages::MoveTo>("close_hand", gripper_planner);
          stage->setGroup(hand);
          stage->setGoal("close");
          grasp->insert(std::move(stage));
        }

        left_robot->insert(std::move(grasp));
      }

      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("lift_object", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.01, 0.3);
        stage->setIKFrame(tcp);
        stage->setMarkerNS("lift_object");

        geometry_msgs::msg::Vector3Stamped direction;
        direction.header.frame_id = "base";
        direction.vector.z = 1.0;
        stage->setDirection(direction);
        left_robot->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_handover", sampling_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        stage->setGroup(arm);
        stage->setGoal("left_handover_pose");
        left_robot->insert(std::move(stage));
      }

      task.add(std::move(left_robot));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_collision (hand, hand, object)");   
      stage->allowCollisions(task.getRobotModel()->getJointModelGroup("left_fr3_hand")->getLinkModelNamesWithCollisionGeometry(),
                            task.getRobotModel()->getJointModelGroup("right_fr3_hand")->getLinkModelNamesWithCollisionGeometry(),
                            true);
      

      stage->allowCollisions(object_name,
                            task.getRobotModel()->getJointModelGroup("right_fr3_hand")->getLinkModelNamesWithCollisionGeometry(),
                            true);
      
      task.add(std::move(stage));
    }

    /****************************************************
     * RIGHT ARM
     ***************************************************/
    {
      const auto arm = "right_fr3_arm";
      const auto hand = "right_fr3_hand";
      const auto tcp = "right_fr3_hand_tcp";

      {
        auto right_pick = std::make_unique<mtc::SerialContainer>("handover_sequence");
        right_pick->setProperty("group", arm);
        right_pick->setProperty("eef", hand);
        right_pick->setProperty("ik_frame", tcp);

          // Open Hand
        {
          auto stage = std::make_unique<mtc::stages::MoveTo>("open_hand", gripper_planner);
          stage->setGroup(hand);
          stage->setGoal("open");
          right_pick->insert(std::move(stage));
        }

        // Connect to Pick Position
        {
          auto stage = std::make_unique<mtc::stages::Connect>("move_to_handover", mtc::stages::Connect::GroupPlannerVector{{arm, sampling_planner}});
          stage->setTimeout(15.0);
          stage->properties().configureInitFrom(mtc::Stage::PARENT);
          right_pick->insert(std::move(stage));
        }

        // Approach & Handover
        {
          auto handover = std::make_unique<mtc::SerialContainer>("handover");
          right_pick->properties().exposeTo(handover->properties(), {"eef", "group", "ik_frame"});
          handover->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

          // Approach
          {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("handover_approach", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMarkerNS("handover_approach");
            stage->setMinMaxDistance(0.01, 0.1);

            geometry_msgs::msg::Vector3Stamped direction;
            direction.header.frame_id = tcp;
            direction.vector.z = 1.0;
            stage->setDirection(direction);
            handover->insert(std::move(stage));
          }

          // Pose Generation
          /*
          {
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate_handover_pose");
            stage->setPreGraspPose("open");
            stage->setObject(object_name);
            stage->setAngleDelta(M_PI / 2);
            stage->setMonitoredStage(task.stages()->findChild("allow_collision (hand, hand, object)"));
            stage->properties().configureInitFrom(mtc::Stage::PARENT);


            Eigen::Isometry3d handover_frame_transform = Eigen::Isometry3d::Identity();
            
            Eigen::Quaterniond q_x(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
            Eigen::Quaterniond q_y(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
            Eigen::Quaterniond q = q_x * q_y;
            q.normalize();

            handover_frame_transform.linear() = q.matrix();

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("handover_pose_ik", std::move(stage));
            wrapper->setMaxIKSolutions(1);
            wrapper->setIKFrame(handover_frame_transform, tcp);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

            handover->insert(std::move(wrapper));
          }
          */

          {
            auto stage = std::make_unique<mtc::stages::GeneratePose>("generate_handover_pose");
            stage->setMonitoredStage(task.stages()->findChild("allow_collision (hand, hand, object)"));
            stage->properties().configureInitFrom(mtc::Stage::PARENT);

           
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
      
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("handover_pose_ik", std::move(stage));
            wrapper->setMaxIKSolutions(1);
            wrapper->setIKFrame(tcp);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

            handover->insert(std::move(wrapper));
          }
          
          // Transfer Object
          {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("transfer_object)");
            stage->allowCollisions(object_name,
                                  task.getRobotModel()->getJointModelGroup(hand)->getLinkModelNamesWithCollisionGeometry(),
                                  true);

            stage->allowCollisions(task.getRobotModel()->getJointModelGroup("left_fr3_hand")->getLinkModelNamesWithCollisionGeometry(),
                                  task.getRobotModel()->getJointModelGroup(hand)->getLinkModelNamesWithCollisionGeometry(),
                                  true);

            stage->detachObject(object_name, "left_fr3_hand_tcp");
            stage->attachObject(object_name, tcp);
              
            handover->insert(std::move(stage));
          }

          {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close_hand_right", gripper_planner);
            stage->setGroup(hand);
            stage->setGoal("close");
            handover->insert(std::move(stage));
          }

          right_pick->insert(std::move(handover));
        }
     

        // Apertura pinza sinistra
        {
          auto stage = std::make_unique<mtc::stages::MoveTo>("open_hand_left", gripper_planner);
          stage->setGroup("left_fr3_hand");
          stage->setGoal("open");
          right_pick->insert(std::move(stage));
        }

        {
          auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat_left", cartesian_planner);
          stage->setGroup("left_fr3_arm");
          stage->setMarkerNS("retreat_left");
          stage->setMinMaxDistance(0.01, 0.1);
          
          geometry_msgs::msg::Vector3Stamped direction;
          direction.header.frame_id = "left_fr3_hand_tcp";
          direction.vector.z = -1.0;
          stage->setDirection(direction);
          right_pick->insert(std::move(stage));
        }

        task.add(std::move(right_pick));
      }
    
      // PLACE SEQUENCE
      {
        auto right_place = std::make_unique<mtc::SerialContainer>("right_arm_place_sequence");
        right_place->setProperty("group", arm);
        right_place->setProperty("eef", hand);
        right_place->setProperty("ik_frame", tcp);

        {
          auto stage = std::make_unique<mtc::stages::Connect>("move_to_place_approach", mtc::stages::Connect::GroupPlannerVector{{arm, sampling_planner}});
          stage->setTimeout(15.0);
          stage->properties().configureInitFrom(mtc::Stage::PARENT);
          right_place->insert(std::move(stage));
        }

        {
          auto place = std::make_unique<mtc::SerialContainer>("place");
          right_place->properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
          place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

          {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("place_approach", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMarkerNS("place_approach");
            stage->setMinMaxDistance(0.00, 0.3); 
            
            geometry_msgs::msg::Vector3Stamped direction;
            direction.header.frame_id = tcp;
            direction.vector.z = 1.0; 
            stage->setDirection(direction);
            place->insert(std::move(stage));
          }

          {
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate_place_pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->setMonitoredStage(task.stages()->findChild("handover_sequence/retreat_left"));
            stage->setObject(object_name);
            
            auto end_pose_vec = node_->get_parameter("end_pose").as_double_array();
            
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = "base"; 
            
            target_pose.pose.position.x = end_pose_vec[0];
            target_pose.pose.position.y = 1.1;
            target_pose.pose.position.z = 0.25 + 0.30 * end_pose_vec[1];

            Eigen::Quaterniond q_x(Eigen::AngleAxisd(-(M_PI / 2), Eigen::Vector3d::UnitX()));
            Eigen::Quaterniond q_z(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

            Eigen::Quaterniond q = q_x * q_z;
            q.normalize();

            target_pose.pose.orientation.x = q.x();
            target_pose.pose.orientation.y = q.y();
            target_pose.pose.orientation.z = q.z();
            target_pose.pose.orientation.w = q.w();

            stage->setPose(target_pose);

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place_pose_ik", std::move(stage));
            wrapper->setMaxIKSolutions(1);
            wrapper->setIKFrame(tcp);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            place->insert(std::move(wrapper));
          }

          {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open_hand", gripper_planner);
            stage->setGroup(hand);
            stage->setGoal("open");
            place->insert(std::move(stage));
          }

          {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach (object, hand)");
            stage->detachObject(object_name, tcp);
            place->insert(std::move(stage));
          }

          {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat_right", cartesian_planner);
            stage->setGroup(arm);
            stage->setMarkerNS("retreat_right");
            stage->setMinMaxDistance(0.01, 0.1);
            
            geometry_msgs::msg::Vector3Stamped direction;
            direction.header.frame_id = tcp;
            direction.vector.z = -1.0;
            stage->setDirection(direction);
            place->insert(std::move(stage));
          }

          right_place->insert(std::move(place));
        }

        {
          auto stage = std::make_unique<mtc::stages::MoveTo>("ready_pose", sampling_planner);
          stage->setGroup(arm);
          stage->setGoal("ready");
          right_place->insert(std::move(stage));
        }
  
        task.add(std::move(right_place));
      }
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

  rclcpp::sleep_for(std::chrono::seconds(4));
  mtc_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}