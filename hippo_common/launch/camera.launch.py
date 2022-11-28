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

    composable_nodes = [
        launch_ros.descriptions.ComposableNode(
            package='v4l2_camera',
            plugin='v4l2_camera::V4L2Camera',
            name='vertical_camera',
            parameters=[{
                'video_device': '/dev/video0',
                'pixel_format': 'YUYV',
                'output_encoding': 'mono8',
                'image_size': [1280, 800]
            }],
            extra_arguments=[
                {
                    'use_intra_process_comms': True
                },
            ]),
        launch_ros.descriptions.ComposableNode(
            package='image_proc',
            plugin='image_proc::CropDecimateNode',
            name='cropper',
            parameters=[
                {
                    'width': 960,
                    'height': 600,
                    'offset_x': 160,
                    'offset_y': 100
                },
            ],
            remappings=[
                ('out/image_raw', 'cropped/image_raw'),
                ('in/camera_info', 'camera_info'),
                ('out/camera_info', 'cropped/camera_info')
                ('in/image_raw', 'image_raw'),
            ],
            extra_arguments=[
                {
                    'use_intra_process_comms': True
                },
            ]),
        launch_ros.descriptions.ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectifier',
            # Remap subscribers and publishers
            remappings=[
                ('image', 'cropped/image_raw'),
                ('camera_info', 'cropped/camera_info'),
                ('image_rect', 'image_rect'),
            ],
            extra_arguments=[
                {
                    'use_intra_process_comms': True
                },
            ]),
    ]
    container = launch_ros.actions.ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen')

    nodes_group = launch.actions.GroupAction([
        launch_ros.actions.PushRosNamespace(
            launch.substitutions.LaunchConfiguration('vehicle_name')),
        launch_ros.actions.PushRosNamespace(
            launch.substitutions.LaunchConfiguration('camera_name')),
        container,
    ])

    return launch.LaunchDescription(launch_args + [
        nodes_group,
    ])
