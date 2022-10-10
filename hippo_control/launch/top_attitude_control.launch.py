import launch
from ament_index_python.packages import get_package_share_path
import launch_ros


def generate_launch_description():
    package_name = 'hippo_control'
    package_path = get_package_share_path(package_name)
    default_vehicle_name = 'uuv00'
    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value=default_vehicle_name,
        description='Vehicle name used as namespace.')

    launch_paths = [
        str(package_path / 'launch/node_actuator_mixer.launch.py'),
        str(package_path / 'launch/node_attitude_control.launch.py'),
    ]
    launch_includes = [
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(x))
        for x in launch_paths
    ]

    namespace_group = launch.actions.GroupAction([
        launch_ros.actions.PushRosNamespace(
            launch.substitutions.LaunchConfiguration('vehicle_name'))
    ] + launch_includes)

    launch_descriptions = [vehicle_name_launch_arg, namespace_group]

    return launch.LaunchDescription(launch_descriptions)
