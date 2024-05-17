from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
)


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(launch_description=launch_description)
    action = DeclareLaunchArgument(
        'carrot_control_config',
        description='Path to the carrot control YAML file',
    )
    launch_description.add_action(action)


def add_carrot_control_node(launch_description: LaunchDescription):
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    action = Node(
        package='hippo_control',
        executable='carrot_control_node',
        parameters=[
            args,
            LaunchConfiguration('carrot_control_config'),
        ],
    )
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    add_carrot_control_node(launch_description=launch_description)

    return launch_description
