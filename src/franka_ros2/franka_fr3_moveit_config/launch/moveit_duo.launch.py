import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    Shutdown
)
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    left_robot_ip_parameter_name = 'left_robot_ip'
    right_robot_ip_parameter_name = 'right_robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    ee_id_parameter_name = 'ee_id'
    use_sim_parameter_name = 'use_sim'

    left_robot_ip = LaunchConfiguration(left_robot_ip_parameter_name)
    right_robot_ip = LaunchConfiguration(right_robot_ip_parameter_name)
    
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    ee_id = LaunchConfiguration(ee_id_parameter_name)
    use_sim = LaunchConfiguration(use_sim_parameter_name)

    # Command-line arguments

    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )

    # planning_context
    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'),'robots', 'fr3_duo', 'fr3_duo.urdf.xacro')

    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', 
            franka_xacro_file, 
            ' hand:=', load_gripper,
            ' robot_ips:="[\'', left_robot_ip, '\', \'', right_robot_ip, '\']"',
            ' ee_id:=', ee_id, 
            ' with_sc:=true' ,
            ' ros2_control:=true',
            ' use_sim:=', use_sim])
    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}


    franka_semantic_xacro_file = os.path.join(get_package_share_directory('franka_description'),'robots', 'fr3_duo', 'fr3_duo.srdf.xacro')

    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file,
         ' hand:=', load_gripper, 
         ' ee_id:=', ee_id, 
         ' arm_prefix_robot_1:=left_',
         ' arm_prefix_robot_2:=right_', 
        ])
    
    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}

    
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics_duo.yaml')
    
    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning_duo.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml('franka_fr3_moveit_config', 'config/fr3_duo_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    mtc_capabilities = {'capabilities': 'move_group/ExecuteTaskSolutionCapability'}
    
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            mtc_capabilities,
            {"use_sim_time": use_sim}
        ],
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory('franka_fr3_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'franka_duo.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )


    ros2_controllers_path = os.path.join(get_package_share_directory('franka_fr3_moveit_config'),'config','fr3_duo_ros_controllers.yaml')

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='franka_duo',
        parameters=[robot_description, ros2_controllers_path],
        remappings=[
            ('~/robot_description', '/robot_description'),
            ('joint_states', 'franka_duo/joint_states')
        ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    # Load controllers
    load_controllers = []
    for controller in ['left_arm_controller', 'right_arm_controller', 'joint_state_broadcaster']:
        load_controllers.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner', controller,
                    '--controller-manager-timeout', '60',
                    '--controller-manager', '/franka_duo/controller_manager'
                ],
                output='screen'
            )
        )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'source_list': [
                'franka_duo/left/arm/joint_states',
                'franka_duo/right/arm/joint_states',
                'franka_duo/left_gripper/joint_states',
                'franka_duo/right_gripper/joint_states',
            ], 'rate': 30}],
    )

    franka_robot_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['franka_robot_state_broadcaster'],
        output='screen',
        condition=UnlessCondition(use_sim), # Solo sul robot reale
    )
    

    left_robot_arg = DeclareLaunchArgument(
        left_robot_ip_parameter_name,
        default_value='172.16.0.2',
        description='Hostname or IP address of the left robot.')

    right_robot_arg = DeclareLaunchArgument(
        right_robot_ip_parameter_name,
        default_value='172.16.0.3',
        description='Hostname or IP address of the right robot.')
    
    load_gripper_arg = DeclareLaunchArgument(
        load_gripper_parameter_name,
        default_value='true',
        description='Whether to load the gripper or not (true or false)'
    )

    ee_id_arg = DeclareLaunchArgument(
        ee_id_parameter_name,
        default_value='franka_hand',
        description='The end-effector id to use. Available options: none, franka_hand'
    )

    use_sim_arg = DeclareLaunchArgument(
        use_sim_parameter_name,
        default_value='false',
        description='Is the robot being simulated?.'
    )

    gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('franka_gripper'), 'launch', 'gripper_duo.launch.py'])]),
            launch_arguments={left_robot_ip_parameter_name: left_robot_ip,
                              right_robot_ip_parameter_name: right_robot_ip,
                              use_sim_parameter_name: use_sim,
                            }.items(),
    )

    return LaunchDescription(
        [left_robot_arg,
         right_robot_arg,
         load_gripper_arg,
         ee_id_arg,
         db_arg,
         use_sim_arg,
         rviz_node,
         robot_state_publisher,
         run_move_group_node,
         ros2_control_node,
         joint_state_publisher,
         franka_robot_state_broadcaster,
         gripper_launch_file
         ]
        + load_controllers
    )
