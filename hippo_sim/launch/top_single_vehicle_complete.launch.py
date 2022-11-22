from ament_index_python.packages import get_package_share_path
import launch


def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
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
    return launch.LaunchDescription([] + launch_descriptions)
