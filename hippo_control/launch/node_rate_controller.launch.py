from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hippo_common.launch_helper import declare_use_sim_time


def declare_launch_args(launch_description: LaunchDescription):
    declare_use_sim_time(launch_description=launch_description)

    action = DeclareLaunchArgument('rate_control_config')
    launch_description.add_action(action)


def add_node(launch_description: LaunchDescription):
    action = Node(
        package='hippo_control',
        executable='rate_controller_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('rate_control_config'),
        ],
        output='screen',
    )
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    add_node(launch_description=launch_description)
    return launch_description
