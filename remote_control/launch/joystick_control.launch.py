import launch
import launch_ros


def generate_launch_description():
    launch_args = [launch.actions.DeclareLaunchArgument(name='vehicle_name')]
    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')

    composable_nodes = [
        launch_ros.descriptions.ComposableNode(
            package='joy',
            plugin='joy::Joy',
            name='joystick',
            parameters=[{
                'device_id': 0,
                'device_name': '',
                'deadzone': 0.5,
                'autorepeat_rate': 20.0,
                'sticky_buttons': False,
                'coalesce_interval_ms': 1,
            }],
            extra_arguments=[
                {
                    'use_intra_process_comms': True
                },
            ]),
        launch_ros.descriptions.ComposableNode(
            package='remote_control',
            plugin='remote_control::joystick::JoyStick',
            name='joystick_mapper',
            extra_arguments=[
                {
                    'use_intra_process_comms': True
                },
            ]),
    ]
    container = launch_ros.actions.ComposableNodeContainer(
        name='joystick_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen')

    nodes_group = launch.actions.GroupAction([
        launch_ros.actions.PushRosNamespace(vehicle_name),
        container,
    ])

    return launch.LaunchDescription(launch_args + [
        nodes_group,
    ])
