from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from hippo_common.launch_helper import LaunchArgsDict


def declare_launch_args(launch_description: LaunchDescription):
    package_path = get_package_share_path('hippo_control')

    default_path = str(
        package_path
        / 'config/attitude_control/geometric_hippocampus_default.yaml'
    )
    action = DeclareLaunchArgument(
        name='attitude_control_config', default_value=default_path
    )
    launch_description.add_action(action)

    default_path = str(
        package_path / 'config/actuator_mixer/hippocampus_default.yaml'
    )
    action = DeclareLaunchArgument(
        name='mixer_path',
        default_value=default_path,
        description='Path to mixer configuration .yaml file.',
    )
    launch_description.add_action(action)


def include_launch_files(launch_description: LaunchDescription):
    package_path = get_package_share_path('hippo_control')

    path = str(
        package_path
        / 'launch/attitude_control/attitude_control_generic.launch.py'
    )
    source = PythonLaunchDescriptionSource(path)
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args.add(['mixer_path', 'attitude_control_config'])
    action = IncludeLaunchDescription(source, launch_arguments=args.items())
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    include_launch_files(launch_description=launch_description)

    return launch_description
