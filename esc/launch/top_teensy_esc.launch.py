import launch
import launch_ros
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    package_name = 'esc'
    package_path = get_package_share_path(package_name)

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')
    serial_port = launch.substitutions.LaunchConfiguration('serial_port')

    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='decide if simulation time is used',
    )
    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name', description='Vehicle name used as namespace.'
    )
    serial_port_arg = launch.actions.DeclareLaunchArgument(
        name='serial_port', default_value='/dev/teensy_data'
    )

    launch_includes = []
    # include teensy_esc node
    path = str(package_path / 'launch/node_teensy_esc.launch.py')
    launch_arguments = {
        'serial_port': serial_port,
        'use_sim_time': use_sim_time,
        'vehicle_name': vehicle_name,
    }.items()
    descr = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(path),
        launch_arguments=launch_arguments,
    )

    launch_includes.append(descr)

    namespace_group = launch.actions.GroupAction(
        [launch_ros.actions.PushRosNamespace(vehicle_name)] + launch_includes
    )

    return launch.LaunchDescription(
        [
            use_sim_time_launch_arg,
            serial_port_arg,
            vehicle_name_launch_arg,
            namespace_group,
        ]
    )
