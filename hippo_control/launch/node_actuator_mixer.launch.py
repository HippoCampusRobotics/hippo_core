from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_name = 'hippo_control'
    package_path = get_package_share_path(package_name)
    default_mixer_path = package_path / (
        'config/actuator_mixer_hippocampus_default.yaml')
    mixer_launch_arg = launch.actions.DeclareLaunchArgument(
        name='mixer_path',
        default_value=str(default_mixer_path),
        description='Path to the mixer configuration .yaml file.')

    return launch.LaunchDescription([
        mixer_launch_arg,
        launch_ros.actions.Node(
            package=package_name,
            executable='actuator_command_mixer_node',
            parameters=[launch.substitutions.LaunchConfiguration('mixer_path')],
            output='screen'),
    ])
