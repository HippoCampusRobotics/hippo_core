from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    world = package_path / 'models' / 'world' / 'empty.sdf'
    pool_path = package_path / 'models/pool/urdf/pool.xacro'

    pool_description = launch.substitutions.LaunchConfiguration(
        'pool_description',
        default=launch.substitutions.Command([
            'ros2 run hippo_sim create_robot_description.py ',
            '--input ',
            str(pool_path),
        ]))
    pool_params = {'pool_description': pool_description}

    gazebo = launch.actions.ExecuteProcess(
        cmd=['ign', 'gazebo', '-v 3', str(world)], output='screen')

    return launch.LaunchDescription([
        gazebo,
        launch_ros.actions.Node(package='hippo_sim',
                                executable='spawn',
                                parameters=[pool_params],
                                arguments=[
                                    '--param',
                                    'pool_description',
                                    '--x',
                                    '1.0',
                                    '--y',
                                    '2.0',
                                    '--z',
                                    '-1.5',
                                ],
                                output='screen'),
        launch.actions.
        RegisterEventHandler(event_handler=launch.event_handlers.OnProcessExit(
            target_action=gazebo,
            on_exit=[launch.actions.EmitEvent(
                event=launch.events.Shutdown())])),
    ])
