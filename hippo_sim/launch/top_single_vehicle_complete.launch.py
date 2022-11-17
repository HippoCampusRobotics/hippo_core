from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    fake_estimator_launch_arg = launch.actions.DeclareLaunchArgument(
        name='fake_state_estimation', default_value='false')
    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name', default_value='uuv00')
    fake_estimator_node = launch_ros.actions.Node(
        package='hippo_sim',
        executable='fake_state_estimator',
        name='state_estimator',
        namespace=launch.substitutions.LaunchConfiguration('vehicle_name'),
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration('fake_state_estimation')))
    include_paths = [
        str(package_path / 'launch/start_gazebo.launch.py'),
        # str(package_path / 'launch/spawn_apriltag_floor.launch.py'),
        str(package_path / 'launch/spawn_vehicle.launch.py'),
    ]
    launch_descriptions = [
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(x))
        for x in include_paths
    ]
    return launch.LaunchDescription([
        fake_estimator_launch_arg,
        vehicle_name_launch_arg,
        fake_estimator_node,
    ] + launch_descriptions)
