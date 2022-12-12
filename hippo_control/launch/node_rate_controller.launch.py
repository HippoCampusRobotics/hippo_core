from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_name = 'hippo_control'
    package_path = get_package_share_path(package_name)
    default_config_path = package_path / (
        'config/rate_control_hippocampus_default.yaml')
    config_launch_arg = launch.actions.DeclareLaunchArgument(
        name='rate_control_config',
        default_value=str(default_config_path),
        description='Path to the rate configuration .yaml file.')
    return launch.LaunchDescription([
        config_launch_arg,
        launch_ros.actions.Node(
            package=package_name,
            executable='rate_controller_node',
            parameters=[
                launch.substitutions.LaunchConfiguration('rate_control_config'),
            ],
            output='screen'),
    ])
