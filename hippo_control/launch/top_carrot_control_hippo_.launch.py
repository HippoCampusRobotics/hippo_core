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


def declare_launch_args(launch_description: LaunchDescription):
    action = DeclareLaunchArgument(name='vehicle_name')
    launch_description.add_action(action)
    action = DeclareLaunchArgument(name='use_sim_time')
    launch_description.add_action(action)


def include_mixer(launch_description: LaunchDescription):
    package_path = get_package_share_path('hippo_control')

    path = str(package_path / 'launch/node_actuator_mixer.launch.py')
    source = PythonLaunchDescriptionSource(path)
    action = IncludeLaunchDescription(source,
                                      launch_arguments={
                                          'vehicle_name':
                                          LaunchConfiguration('vehicle_name'),
                                          'use_sim_time':
                                          LaunchConfiguration('use_sim_time'),
                                      }.items())
    action = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        action,
    ])
    launch_description.add_action(action)


def include_attitude_control(launch_description: LaunchDescription):
    package_path = get_package_share_path('hippo_control')
    path = str(package_path / 'launch/node_attitude_control.launch.py')
    source = PythonLaunchDescriptionSource(path)
    action = IncludeLaunchDescription(source,
                                      launch_arguments={
                                          'vehicle_name':
                                          LaunchConfiguration('vehicle_name'),
                                          'use_sim_time':
                                          LaunchConfiguration('use_sim_time'),
                                      }.items())
    action = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        action,
    ])
    launch_description.add_action(action)


def include_carrot_control(launch_description: LaunchDescription):
    package_path = get_package_share_path('hippo_control')
    path = str(package_path / 'config/carrot_control_hippocampus.yaml')
    action = DeclareLaunchArgument('carrot_control_config', default_value=path)
    launch_description.add_action(action)
    path = str(package_path / 'launch/node_carrot_control.launch.py')
    source = PythonLaunchDescriptionSource(path)
    action = IncludeLaunchDescription(
        source,
        launch_arguments={
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'carrot_control_config':
            LaunchConfiguration('carrot_control_config')
        }.items())
    action = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        action,
    ])
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    include_mixer(launch_description=launch_description)
    include_attitude_control(launch_description=launch_description)
    include_carrot_control(launch_description=launch_description)

    return launch_description
