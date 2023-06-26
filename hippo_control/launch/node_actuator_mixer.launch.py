from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_name = 'hippo_control'
    package_path = get_package_share_path(package_name)
    default_mixer_path = package_path / (
        'config/actuator_mixer_hippocampus_default.yaml')

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')


    mixer_launch_arg = launch.actions.DeclareLaunchArgument(
        name='mixer_path',
        default_value=str(default_mixer_path),
        description='Path to the mixer configuration .yaml file.')
    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value=str(False),
        description='decide if simulation time is used'
    )
    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value='bluerov',
        description='used for node namespace'
    )
    return launch.LaunchDescription([
        use_sim_time_launch_arg,
        vehicle_name_launch_arg,
        mixer_launch_arg,
        launch_ros.actions.Node(
            package=package_name,
            executable='actuator_mixer_node',
            namespace=vehicle_name,
            parameters=[{'use_sim_time': use_sim_time}, launch.substitutions.LaunchConfiguration('mixer_path')],
            output='screen'),
    ])
