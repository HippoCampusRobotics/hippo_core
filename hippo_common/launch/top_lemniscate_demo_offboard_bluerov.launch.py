from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hippo_common.launch_helper import (
    LaunchArgsDict,
    config_file_path,
    declare_vehicle_name_and_sim_time,
    launch_file_source,
)


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(launch_description)
    pkg = 'hippo_control'
    config_file = config_file_path(pkg, 'actuator_mixer_bluerov_advanced.yaml')
    action = DeclareLaunchArgument('mixer_path', default_value=config_file)
    launch_description.add_action(action)

    config_file = config_file_path(pkg, 'transformations_bluerov_default.yaml')
    action = DeclareLaunchArgument(
        name='tf_vehicle_config_file', default_value=config_file
    )
    launch_description.add_action(action)


def include_path_follower():
    pkg = 'hippo_control'
    source = launch_file_source(
        pkg, 'top_path_following_intra_process.launch.py'
    )
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args.add('mixer_path')
    return IncludeLaunchDescription(source, launch_arguments=args.items())


def include_visual_localization():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    pkg = 'visual_localization'
    source = launch_file_source(pkg, 'top_localization.launch.py')
    return IncludeLaunchDescription(source, launch_arguments=args.items())


def add_tf_publisher_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='hippo_common',
        namespace=LaunchConfiguration('vehicle_name'),
        executable='tf_publisher_vehicle_node',
        parameters=[args, LaunchConfiguration('tf_vehicle_config_file')],
    )


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    actions = [
        include_path_follower(),
        add_tf_publisher_node(),
        include_visual_localization(),
    ]
    for action in actions:
        launch_description.add_action(action)
    return launch_description
