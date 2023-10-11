from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from hippo_common.launch_helper import (
    PassLaunchArguments,
    declare_vehicle_name_and_sim_time,
)


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(launch_description=launch_description)

    package_path = get_package_share_path('hippo_control')
    path = str(package_path / 'config/actuator_mixer/hippocampus_default.yaml')
    action = DeclareLaunchArgument('mixer_path', default_value=path)
    launch_description.add_action(action)

    path = str(package_path / 'config/rate_control_hippocampus_default.yaml')
    action = DeclareLaunchArgument('rate_control_config', default_value=path)
    launch_description.add_action(action)

    path = str(package_path /
               'config/attitude_control/quaternion_hippocampus_default.yaml')
    action = DeclareLaunchArgument('attitude_control_config',
                                   default_value=path)
    launch_description.add_action(action)

    package_path = get_package_share_path('path_planning')
    path = str(package_path / 'config/path_follower_default.yaml')
    action = DeclareLaunchArgument('path_follower_config', default_value=path)
    launch_description.add_action(action)


def add_composable_nodes(launch_description: LaunchDescription):
    nodes = []
    args = PassLaunchArguments()
    args.add_vehicle_name_and_sim_time()
    extra_args = [{'use_intra_process_comms': True}]

    node = ComposableNode(
        package='hippo_control',
        plugin='hippo_control::mixer::ActuatorMixerNode',
        namespace=LaunchConfiguration('vehicle_name'),
        name='actuator_mixer',
        parameters=[
            args,
            LaunchConfiguration('mixer_path'),
        ],
        extra_arguments=extra_args,
    )
    nodes.append(node)

    node = ComposableNode(
        package='hippo_control',
        plugin='hippo_control::rate_control::RateControlNode',
        namespace=LaunchConfiguration('vehicle_name'),
        name='rate_controller',
        parameters=[
            args,
            LaunchConfiguration('rate_control_config'),
        ],
        extra_arguments=extra_args,
    )
    nodes.append(node)

    node = ComposableNode(
        package='hippo_control',
        plugin='hippo_control::attitude_control::QuaternionControlNode',
        namespace=LaunchConfiguration('vehicle_name'),
        name='attitude_controller',
        parameters=[
            args,
            LaunchConfiguration('attitude_control_config'),
        ],
        extra_arguments=extra_args,
    )
    nodes.append(node)
    node = ComposableNode(
        package='path_planning',
        plugin='path_planning::PathFollowerNode',
        namespace=LaunchConfiguration('vehicle_name'),
        name='path_follower',
        parameters=[
            args,
            LaunchConfiguration('path_follower_config'),
        ],
        extra_arguments=extra_args,
    )
    nodes.append(node)

    container = ComposableNodeContainer(
        name='path_follower_container',
        namespace=LaunchConfiguration('vehicle_name'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
        output='screen',
    )
    launch_description.add_action(container)


def include_attitude_control(launch_description: LaunchDescription):
    package_path = get_package_share_path('hippo_control')
    path = str(package_path /
               'launch/attitude_control/attitude_control_hippocampus.launch.py')
    source = PythonLaunchDescriptionSource(path)
    args = PassLaunchArguments()
    args.add_vehicle_name_and_sim_time()
    action = IncludeLaunchDescription(source, launch_arguments=args.items())
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    add_composable_nodes(launch_description=launch_description)

    return launch_description
