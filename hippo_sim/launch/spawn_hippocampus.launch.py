from ament_index_python.packages import get_package_share_path
import launch


def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    launch_path = str(package_path / 'launch/spawn_vehicle.launch.py'),
    model_path = str(package_path / 'models/hippo3/urdf/hippo3.xacro')

    vehicle_spawner = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch_path),
        launch_arguments=dict(model_path=model_path).items())
    return launch.LaunchDescription([vehicle_spawner])
