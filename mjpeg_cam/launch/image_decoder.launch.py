from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_description = LaunchDescription()

    action = Node(executable='image_decoder',
                  name='image_decoder',
                  package='mjpeg_cam',
                  namespace=LaunchConfiguration('camera_name'),
                  parameters=[
                      {
                          'use_sim_time': False,
                      },
                  ])
    launch_description.add_action(action)

    return launch_description
