from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace

from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
)


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(launch_description=launch_description)

    package_path = get_package_share_path('hippo_common')
    default_config_vehicle_file = package_path / (
        'config/transformations_bluerov_default.yaml'
    )
    action = DeclareLaunchArgument(
        name='tf_vehicle_config_file',
        default_value=str(default_config_vehicle_file),
    )
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)

    launch_args = LaunchArgsDict()
    launch_args.add_vehicle_name_and_sim_time()

    action = GroupAction(
        [
            PushROSNamespace(LaunchConfiguration('vehicle_name')),
            Node(
                package='hippo_common',
                executable='tf_publisher_vehicle_node',
                output='screen',
                parameters=[
                    launch_args,
                    LaunchConfiguration('tf_vehicle_config_file'),
                ],
            ),
        ]
    )
    launch_description.add_action(action)

    return launch_description
