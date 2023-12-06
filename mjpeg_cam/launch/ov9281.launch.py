from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_description = LaunchDescription()

    pkg_path = get_package_share_path('mjpeg_cam')
    config_file_path = str(pkg_path / 'config/ov9281.yaml')

    action = Node(executable='mjpeg_cam_node',
                  name=LaunchConfiguration('camera_name'),
                  package='mjpeg_cam',
                  namespace=LaunchConfiguration('camera_name'),
                  parameters=[
                      {
                          'use_sim_time': False,
                      },
                      LaunchConfiguration('config_file',
                                          default=config_file_path),
                  ])
    launch_description.add_action(action)

    return launch_description
