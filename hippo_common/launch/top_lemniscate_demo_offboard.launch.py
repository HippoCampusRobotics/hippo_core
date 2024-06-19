"""
This launch file starts all nodes that are not required to run directly on the
board computer to perform the lemniscate demo.

Example:
    ros2 launch hippo_common top_lemniscate_demo_offboard.launch.py \\
    vehicle_name:=uuv02 \\
    use_sim_time:=false
"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
)


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(launch_description)

    package_path = get_package_share_path('hippo_control')
    path = str(
        package_path
        / 'config/actuator_mixer/hippocampus_normalized_default.yaml'
    )
    action = DeclareLaunchArgument('mixer_path', default_value=path)
    launch_description.add_action(action)

    package_path = get_package_share_path('hippo_common')
    default = str(package_path / ('config/transformations_hippo_default.yaml'))
    action = DeclareLaunchArgument(
        name='tf_vehicle_config_file',
        description='TF config file',
        default_value=default,
    )
    launch_description.add_action(action)


def include_path_follower():
    package_path = get_package_share_path('hippo_control')
    path = str(
        package_path / 'launch/top_path_following_intra_process.launch.py'
    )
    source = PythonLaunchDescriptionSource(path)
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args.add('mixer_path')
    return IncludeLaunchDescription(source, launch_arguments=args.items())


def include_visual_localization():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    package_path = get_package_share_path('visual_localization')
    path = str(package_path / 'launch/top_localization.launch.py')
    source = PythonLaunchDescriptionSource(path)
    return IncludeLaunchDescription(source, launch_arguments=args.items())


def add_tf_publisher_vehicle_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='hippo_common',
        namespace=LaunchConfiguration('vehicle_name'),
        executable='tf_publisher_vehicle_node',
        parameters=[args, LaunchConfiguration('tf_vehicle_config_file')],
    )


def add_px4_bridge_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='visual_localization',
        namespace=LaunchConfiguration('vehicle_name'),
        name='px4_bridge',
        parameters=[args],
        output='screen',
        emulate_tty=True,
    )


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    actions = [
        include_path_follower(),
        add_tf_publisher_vehicle_node(),
        add_px4_bridge_node(),
        include_visual_localization(),
    ]
    for action in actions:
        launch_description.add_action(action)
    return launch_description
