import launch
import launch_ros


def generate_launch_description():
    package_name = 'rapid_trajectories'

    return launch.LaunchDescription([
        launch_ros.actions.Node(package=package_name,
                                executable='simple_tracker_node',
                                output='screen',
                                parameters=[
                                    {
                                        'continuous': True
                                    },
                                ]),
    ])
