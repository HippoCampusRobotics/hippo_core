from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_name = 'hippo_common'
    package_path = get_package_share_path(package_name)
    default_config_vehicle_file = package_path / (
        'config/transformations_hippo_default.yaml')

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')

    vehicle_name_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name', description='Vehicle name used as namespace.')
    use_sim_time_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time', description='decide if simulation time is used.')
    tf_config_vehicle_file_arg = launch.actions.DeclareLaunchArgument(
        name='tf_config_vehicle_file',
        description='TF config file.',
        default_value=str(default_config_vehicle_file))

    launch_args = [
        vehicle_name_arg,
        use_sim_time_arg,
        tf_config_vehicle_file_arg,
    ]

    nodes_group = launch.actions.GroupAction([
        launch_ros.actions.PushRosNamespace(vehicle_name),
        launch_ros.actions.Node(package=package_name,
                                executable='tf_publisher_vehicle_node',
                                output='screen',
                                parameters=[
                                    {
                                        'vehicle_name': vehicle_name,
                                        'use_sim_time': use_sim_time,
                                    },
                                    launch.substitutions.LaunchConfiguration(
                                        'tf_config_vehicle_file'),
                                ]),
    ])

    return launch.LaunchDescription(launch_args + [
        nodes_group,
    ])
