import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_robot_nodes(context):
    left_robot_ip = LaunchConfiguration('left_robot_ip').perform(context)
    right_robot_ip = LaunchConfiguration('right_robot_ip').perform(context)
    arm_id = LaunchConfiguration('arm_id').perform(context)
    use_sim = LaunchConfiguration('use_sim').perform(context)

    # Declare the launch argument names
    left_joint_names = [f'left_{arm_id}_finger_joint1', f'left_{arm_id}_finger_joint2']
    right_joint_names = [f'right_{arm_id}_finger_joint1', f'right_{arm_id}_finger_joint2']

    nodes = []

    # Load the gripper configuration file
    gripper_config = os.path.join(get_package_share_directory('franka_gripper'), 'config', 'franka_duo_gripper_node.yaml')
    nodes.append(
        Node(
            package='franka_gripper',
            executable='franka_gripper_node',
            name=['left_gripper'],
            namespace='franka_duo',
            parameters=[{'robot_ip': left_robot_ip, 'joint_names': left_joint_names}, gripper_config],
            condition=UnlessCondition(use_sim),
        )
    )
    nodes.append(
        Node(
            package='franka_gripper',
            executable='franka_gripper_node',
            name=['right_gripper'],
            namespace='franka_duo',
            parameters=[{'robot_ip': right_robot_ip, 'joint_names': right_joint_names}, gripper_config],
            condition=UnlessCondition(use_sim),
        )
    )
    
    
    nodes.append(
        Node(
            package='franka_gripper',
            executable='franka_gripper_node',
            name=['left_gripper'],
            namespace='franka_duo',
            parameters=[{'robot_ip': left_robot_ip, 'joint_names': left_joint_names, 'use_sim': True}, gripper_config],
            condition=IfCondition(use_sim),
        )
    )
    nodes.append(
        Node(
            package='franka_gripper',
            executable='franka_gripper_node',
            name=['right_gripper'],
            namespace='franka_duo',
            parameters=[{'robot_ip': right_robot_ip, 'joint_names': right_joint_names, 'use_sim': True}, gripper_config],
            condition=IfCondition(use_sim),
        )
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'left_robot_ip',
                description='Hostname or IP address of the left robot.'
            ),
            DeclareLaunchArgument(
                'right_robot_ip',
                description='Hostname or IP address of the right robot.'
            ),
            DeclareLaunchArgument(
                'arm_id',
                default_value='fr3',
                description=(
                    'Name of the arm in the URDF file. This is used to generate the joint '
                    'names of the gripper.'
                ),
            ),
             DeclareLaunchArgument(
                'use_sim', 
                default_value='true',
                description=(
                    ' Is the robot being simulated ?'
                ),
            ),
            OpaqueFunction(function=generate_robot_nodes)
        ]
    )
