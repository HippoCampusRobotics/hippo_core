import launch
import launch_ros
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    package_name = 'rapid_trajectories'
    package_path = get_package_share_path(package_name)
    default_single_tracker_config_path = package_path / (
        'config/simple_tracker.yaml')
    single_tracker_config_launch_arg = launch.actions.DeclareLaunchArgument(
        name='single_tracker_config_path',
        default_value=str(default_single_tracker_config_path),
        description='Path to the single tracker configuration .yaml file.')
    return launch.LaunchDescription([
        single_tracker_config_launch_arg,
        launch_ros.actions.Node(package=package_name,
                                executable='simple_tracker_node',
                                parameters=[
                                    launch.substitutions.LaunchConfiguration(
                                        'single_tracker_config_path'),
                                    {
                                        'use_sim_time': False
                                    },
                                ],
                                output='screen'),
    ])
