from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_name = 'hippo_control'
    package_path = get_package_share_path(package_name)
    default_config_path = package_path / (
        'config/attitude_control_hippocampus_default.yaml')
    config_launch_arg = launch.actions.DeclareLaunchArgument(
        name='attitude_control_config',
        default_value=str(default_config_path),
        description='Path to the attitude configuration .yaml file.')
    feedthrough_launch_arg = launch.actions.DeclareLaunchArgument(
        name="attitude_control_feedthrough",
        default_value='false',
        description=('Whether or not to skip the controller and feed through '
                     'roll, pitch and yaw'))

    feedthrough_param = {
        'feedthrough':
        launch.substitutions.LaunchConfiguration('attitude_control_feedthrough')
    }
    return launch.LaunchDescription([
        config_launch_arg,
        feedthrough_launch_arg,
        launch_ros.actions.Node(package=package_name,
                                executable='attitude_control_node',
                                parameters=[
                                    launch.substitutions.LaunchConfiguration(
                                        'attitude_control_config'),
                                    feedthrough_param,
                                ],
                                output='screen'),
    ])
