from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushROSNamespace

from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
)


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(launch_description=launch_description)


def include_vertical_camera_node():
    package_path = get_package_share_path('mjpeg_cam')
    path = str(package_path / 'launch/ov9281.launch.py')
    source = PythonLaunchDescriptionSource(path)
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return IncludeLaunchDescription(source, launch_arguments=args.items())


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    action = GroupAction(
        [
            PushROSNamespace(LaunchConfiguration('vehicle_name')),
            include_vertical_camera_node(),
        ],
        launch_configurations={'camera_name': 'vertical_camera'},
    )
    launch_description.add_action(action)
    return launch_description
