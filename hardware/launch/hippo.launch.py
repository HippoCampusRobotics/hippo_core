from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace

from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
)


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(launch_description=launch_description,
                                      use_sim_time_default='false')


def include_vertical_camera_node():
    package_path = get_package_share_path('mjpeg_cam')
    path = str(package_path / 'launch/ov9281.launch.py')
    source = PythonLaunchDescriptionSource(path)
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
            add_micro_xrce_agent(),
            add_mavlink_routerd(),
        ],
        launch_configurations={'camera_name': 'vertical_camera'})
    launch_description.add_action(action)
    return launch_description
