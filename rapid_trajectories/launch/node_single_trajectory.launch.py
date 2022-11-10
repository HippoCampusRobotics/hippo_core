import launch
import launch_ros


def generate_launch_description():
    package_name = 'rapid_trajectories'

    return launch.LaunchDescription([
        launch_ros.actions.Node(package=package_name,
                                executable='single_tracker_node',
                                output='screen'),
    ])
