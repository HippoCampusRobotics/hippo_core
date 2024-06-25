from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
    launch_file_source,
)


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(launch_description=launch_description)


def include_vertical_camera_node():
    source = launch_file_source('mjpeg_cam', 'ov9281.launch.py')
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args['camera_name'] = 'vertical_camera'
    return IncludeLaunchDescription(source, launch_arguments=args.items())


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    action = include_vertical_camera_node()
    launch_description.add_action(action)
    return launch_description
