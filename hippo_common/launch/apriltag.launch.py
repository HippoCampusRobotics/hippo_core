from ament_index_python.packages import get_package_share_path
import launch
import launch_ros
import os


def generate_launch_description():

    launch_args = [
        launch.actions.DeclareLaunchArgument(
            name='grid_size',
            default_value='7x4',
            description='Grid size of inner corners of the calibration pattern.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='square_size',
            default_value='0.03',
            description='Edge length of a single square [m].'),
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
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectifier',
            # Remap subscribers and publishers
            remappings=[('image', 'image_offboard'),
                        ('camera_info', 'camera_info'),
                        ('image_rect', 'image_rect')],
        ),
    ]
    container = launch_ros.actions.ComposableNodeContainer(
        name='image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen')
    republish_node = launch_ros.actions.Node(package='image_transport',
                                             executable='republish',
                                             arguments=['compressed', 'raw'],
                                             remappings=[
                                                 ('in/compressed',
                                                  'image_raw/compressed'),
                                                 ('out', 'image_offboard'),
                                             ])
    apriltag_settings = os.path.join(get_package_share_path('apriltag_ros'),
                                     'config', 'settings.param.yaml')
    tags = os.path.join(get_package_share_path('apriltag_ros'), 'config',
                        'tag_bundle.yaml')
    apriltag_node = launch_ros.actions.Node(
        package='apriltag_ros',
        executable='apriltag_ros_continuous_detector_node',
        remappings=[
            ('~/image_rect', 'image_rect'),
            ('~/camera_info', 'camera_info'),
        ],
        parameters=[apriltag_settings, tags],
        output='screen')

    nodes_group = launch.actions.GroupAction([
        launch_ros.actions.PushRosNamespace(
            launch.substitutions.LaunchConfiguration('vehicle_name')),
        launch_ros.actions.PushRosNamespace(
            launch.substitutions.LaunchConfiguration('camera_name')),
        republish_node,
        container,
        apriltag_node,
    ])

    return launch.LaunchDescription(launch_args + [
        nodes_group,
    ])
