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

    package_name = 'hippo_common'
    package_path = get_package_share_path(package_name)
    default_config_vehicle_file = package_path / (
        'config/transformations_hippo_default.yaml')
    action = DeclareLaunchArgument(
        name='tf_vehicle_config_file',
        description='TF config file.',
        default_value=str(default_config_vehicle_file))
    launch_description.add_action(action)


def create_tf_publisher_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='hippo_common',
        executable='tf_publisher_vehicle_node',
        output='screen',
        emulate_tty=True,
        parameters=[args, LaunchConfiguration('tf_vehicle_config_file')],
    )


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)

    action = GroupAction([
        PushROSNamespace(LaunchConfiguration('vehicle_name')),
        create_tf_publisher_node(),
    ])
    launch_description.add_action(action)

    return launch_description
