from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hippo_common.launch_helper import LaunchArgsDict


def declare_launch_args(launch_description: LaunchDescription):
    default = (
        get_package_share_path('path_planning')
        / 'config/path_follower_default.yaml'
    )
    action = DeclareLaunchArgument(
        'path_follower_config',
        description='Path to the path follower configuration YAML file',
        default_value=str(default),
    )
    launch_description.add_action(action)


def add_path_follower_node(launch_description: LaunchDescription):
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    action = Node(
        package='path_planning',
        executable='path_follower_node',
        parameters=[
            args,
            LaunchConfiguration('path_follower_config'),
        ],
    )
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    add_path_follower_node(launch_description=launch_description)

    return launch_description
