from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from hippo_common.launch_helper import declare_vehicle_name_and_sim_time
from hippo_common.launch_helper import PassLaunchArguments


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(launch_description=launch_description)

    package_path = get_package_share_path('hippo_control')
    path = str(package_path / 'config/carrot_control_hippocampus.yaml')
    action = DeclareLaunchArgument('carrot_control_config', default_value=path)
    launch_description.add_action(action)


def include_attitude_control(launch_description: LaunchDescription):
    package_path = get_package_share_path('hippo_control')
    path = str(package_path /
               'launch/attitude_control/attitude_control_hippocampus.launch.py')
    source = PythonLaunchDescriptionSource(path)
    args = PassLaunchArguments()
    args.add_vehicle_name_and_sim_time()
    action = IncludeLaunchDescription(source, launch_arguments=args.items())
    launch_description.add_action(action)


def include_carrot_control(launch_description: LaunchDescription):
    package_path = get_package_share_path('hippo_control')
    path = str(package_path / 'launch/node_carrot_control.launch.py')
    source = PythonLaunchDescriptionSource(path)
    args = PassLaunchArguments()
    args.add_vehicle_name_and_sim_time()
    args.add(['carrot_control_config'])
    action = IncludeLaunchDescription(
        source,
        launch_arguments=args.items())
    action = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        action,
    ])
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    include_attitude_control(launch_description=launch_description)
    include_carrot_control(launch_description=launch_description)

    return launch_description
