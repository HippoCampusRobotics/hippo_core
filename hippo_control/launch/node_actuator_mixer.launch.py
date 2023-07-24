from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from hippo_common.launch_helper import declare_use_sim_time


def declare_launch_args(launch_description: LaunchDescription):
    declare_use_sim_time(launch_description=launch_description)

    default_path = get_package_share_path(
        'hippo_control') / 'config/actuator_mixer/hippocampus_default.yaml'
    action = DeclareLaunchArgument(
        name='mixer_path',
        default_value=str(default_path),
        description='Path to mixer configuration .yaml file.')
    launch_description.add_action(action)


def add_node(launch_description: LaunchDescription):
    action = Node(package='hippo_control',
                  executable='actuator_mixer_node',
                  parameters=[
                      {
                          'use_sim_time': LaunchConfiguration('use_sim_time')
                      },
                      LaunchConfiguration('mixer_path'),
                  ],
                  output='screen')
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    add_node(launch_description=launch_description)
    return launch_description
