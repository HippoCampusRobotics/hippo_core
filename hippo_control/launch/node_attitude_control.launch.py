from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def declare_launch_args(launch_description: LaunchDescription):
    action = DeclareLaunchArgument(
        'attitude_control_feedthrough',
        default_value='false',
        description=(
            'Whether or not to skip the controller and feed through '
            'roll, pitch and yaw.'
        ),
    )
    launch_description.add_action(action)

    default_path = (
        get_package_share_path('hippo_control')
        / 'config/attitude_control_hippocampus_default.yaml'
    )
    action = DeclareLaunchArgument(
        name='attitude_control_config', default_value=str(default_path)
    )
    launch_description.add_action(action)


def add_node(launch_description: LaunchDescription):
    action = Node(
        package='hippo_control',
        executable='geometric_attitude_control_node',
        parameters=[
            {
                'feedthrough': LaunchConfiguration(
                    'attitude_control_feedthrough'
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
            LaunchConfiguration('attitude_control_config'),
        ],
        output='screen',
    )
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    add_node(launch_description=launch_description)
    return launch_description
