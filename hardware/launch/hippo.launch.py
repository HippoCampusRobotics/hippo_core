from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace

from hippo_common.launch_helper import (
    LaunchArgsDict,
    config_file_path,
    declare_vehicle_name_and_sim_time,
    launch_file_source,
)


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(
        launch_description=launch_description, use_sim_time_default='false'
    )
    pkg = 'hippo_control'
    config_file = config_file_path(
        pkg, 'actuator_mixer/hippocampus_normalized_default.yaml'
    )
    action = DeclareLaunchArgument('mixer_path', default_value=config_file)
    launch_description.add_action(action)


def add_mixer_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='hippo_control',
        executable='actuator_mixer_node',
        parameters=[
            args,
            LaunchConfiguration('mixer_path'),
        ],
        output='screen',
    )


def include_vertical_camera_node():
    pkg = 'mjpeg_cam'
    source = launch_file_source(pkg, 'ov9281.launch.py')
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return IncludeLaunchDescription(source, launch_arguments=args.items())


def add_micro_xrce_agent():
    action = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent',
            'serial',
            '--dev',
            '/dev/fcu_data',
            '-b',
            '921600',
        ],
        output='screen',
        emulate_tty=True,
    )
    return action


def add_nsh_node():
    return Node(
        executable='nsh_node',
        package='hardware',
    )


def add_mavlink_routerd():
    action = ExecuteProcess(
        cmd=['mavlink-routerd'],
        output='screen',
        emulate_tty=True,
        # respan required because a FCU reboot will kill mavlink-routerd
        respawn=True,
        respawn_delay=5.0,
    )
    return action


def add_esc_commander():
    return Node(
        executable='esc_commander_node',
        package='esc',
    )


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    action = GroupAction(
        [
            PushROSNamespace(LaunchConfiguration('vehicle_name')),
            include_vertical_camera_node(),
            add_esc_commander(),
            add_mixer_node(),
            add_micro_xrce_agent(),
            add_mavlink_routerd(),
            add_nsh_node(),
        ],
        launch_configurations={'camera_name': 'vertical_camera'},
    )
    launch_description.add_action(action)
    return launch_description
