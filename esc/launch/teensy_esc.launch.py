from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hippo_common.launch_helper import (
    LaunchArgsDict,
    config_file_path,
    declare_vehicle_name_and_sim_time,
)


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(launch_description=launch_description)

    default_path = config_file_path('esc', 'teensy_config.yaml')
    action = DeclareLaunchArgument(
        name='esc_config_file', default_value=default_path
    )
    launch_description.add_action(action)


def add_esc_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    action = Node(
        executable='teensy_commander_node',
        package='esc',
        name='esc_commander',
        namespace=LaunchConfiguration('vehicle_name'),
        parameters=[args, LaunchConfiguration('esc_config_file')],
        output='screen',
        emulate_tty=True,
    )
    return action


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    actions = [
        add_esc_node(),
    ]
    for action in actions:
        launch_description.add_action(action)

    return launch_description
