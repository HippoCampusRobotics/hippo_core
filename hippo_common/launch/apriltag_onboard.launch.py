from ament_index_python.packages import get_package_share_path
import launch
import launch_ros
import os


def generate_launch_description():

    launch_args = [
        launch.actions.DeclareLaunchArgument(
            name='camera_name',
            default_value='vertical_camera',
            description='The name of the camera.',
        ),
        launch.actions.DeclareLaunchArgument(
            name='vehicle_name',
            default_value='uuv00',
            description='Vehicle name used as top level namespace.'),
    ]
    apriltag_settings = os.path.join(get_package_share_path('apriltag_ros'),
                                     'config', 'settings.param.yaml')
    tags = os.path.join(get_package_share_path('apriltag_ros'), 'config',
                        'tag_bundle.yaml')
    apriltag_node = launch_ros.actions.Node(
        package='apriltag_ros',
        executable='apriltag_ros_continuous_detector_node',
        name='apriltag_node',
        remappings=[
            ('~/image_rect', 'image_rect'),
            ('~/camera_info', 'cropped/camera_info'),
        ],
        parameters=[apriltag_settings, tags],
        output='screen')

    nodes_group = launch.actions.GroupAction([
        launch_ros.actions.PushRosNamespace(
            launch.substitutions.LaunchConfiguration('vehicle_name')),
        launch_ros.actions.PushRosNamespace(
            launch.substitutions.LaunchConfiguration('camera_name')),
        apriltag_node,
    ])

    return launch.LaunchDescription(launch_args + [
        nodes_group,
    ])
