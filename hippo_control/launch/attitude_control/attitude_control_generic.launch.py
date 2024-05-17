from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
)


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(launch_description=launch_description)


def include_launch_files(launch_description: LaunchDescription):
    package_path = get_package_share_path('hippo_control')

    path = str(package_path / 'launch/node_actuator_mixer.launch.py')
    source = PythonLaunchDescriptionSource(path)
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args.add(['mixer_path'])
    mixer = IncludeLaunchDescription(source, launch_arguments=args.items())

    path = str(package_path / 'launch/node_attitude_control.launch.py')
    source = PythonLaunchDescriptionSource(path)
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args.add(['attitude_control_config'])
    attitude_controller = IncludeLaunchDescription(
        source,
        launch_arguments=args.items(),
    )

    action = GroupAction(
        [
            PushRosNamespace(LaunchConfiguration('vehicle_name')),
            mixer,
            attitude_controller,
        ]
    )
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    include_launch_files(launch_description=launch_description)

    return launch_description
