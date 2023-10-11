import launch
import launch_ros


def generate_launch_description():
    package_name = 'esc'

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')
    serial_port = launch.substitutions.LaunchConfiguration('serial_port')

    use_sim_time_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time', description='decide if simulation time is used.')
    vehicle_name_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name', description='Vehicle name used as namespace.')
    serial_port_arg = launch.actions.DeclareLaunchArgument(
        name='serial_port',
        description=('Path to the serial device, e.g. "/dev/ttyUSB0" or '
                     '"/dev/teensy_data".'))

    return launch.LaunchDescription([
        use_sim_time_arg,
        vehicle_name_arg,
        serial_port_arg,
        launch_ros.actions.Node(package=package_name,
                                executable='teensy_commander_node',
                                parameters=[
                                    {
                                        'use_sim_time': use_sim_time,
                                        'serial_port': serial_port,
                                        'vehicle_name': vehicle_name,
                                    },
                                ],
                                output='screen'),
    ])
