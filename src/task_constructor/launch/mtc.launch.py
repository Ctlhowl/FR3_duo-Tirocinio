import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    
def prefix_joint_limits(limits_dict, prefixes):
    prefixed_limits = {}
    for joint_name, limits in limits_dict.items():
        for prefix in prefixes:
            prefixed_limits[f'{prefix}{joint_name}'] = limits
    return prefixed_limits

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

    raw_joint_limits = load_yaml('franka_description', 'robots/fr3/joint_limits.yaml')
    prefixed_joint_limits = prefix_joint_limits(raw_joint_limits, ['left_fr3_', 'right_fr3_'])

    joint_limits = {'robot_description_planning': {'joint_limits': prefixed_joint_limits}}
    
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

    run_mtc_node = Node(
        package='task_constructor',
        executable='mtc_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits,
            ompl_planning_pipeline_config,
            {"use_sim_time": True}
        ],
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
    return LaunchDescription([
        left_robot_arg,
        right_robot_arg,
        load_gripper_arg,
        ee_id_arg,
        use_sim_arg,
        run_mtc_node
    ])