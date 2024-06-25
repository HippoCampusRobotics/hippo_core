from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hippo_common.launch_helper import (
    LaunchArgsDict,
    config_file_path,
    declare_vehicle_name_and_sim_time,
)


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(
        launch_description=launch_description, use_sim_time_default='false'
    )

    pkg = 'mjpeg_cam'
    config_file = config_file_path(pkg, 'ov9281.yaml')
    action = DeclareLaunchArgument(
        'camera_config_file', default_value=config_file
    )
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description)

    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    action = Node(
        executable='mjpeg_cam_node',
        name=LaunchConfiguration('camera_name'),
        namespace=[
            LaunchConfiguration('vehicle_name'),
            '/',
            LaunchConfiguration('camera_name'),
        ],
        package='mjpeg_cam',
        parameters=[
            args,
            LaunchConfiguration('camera_config_file'),
        ],
    )
    launch_description.add_action(action)

    return launch_description
