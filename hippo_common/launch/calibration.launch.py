from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


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
        launch.actions.DeclareLaunchArgument(
            name='radial_distortion_coeffs',
            default_value='3',
            description=(
                'Number of radial distortion coefficients. Between 2 and 6')),
    ]

    calibration_args = [
        '-s',
        launch.substitutions.LaunchConfiguration('grid_size'),
        '-q',
        launch.substitutions.LaunchConfiguration('square_size'),
        '-k',
        launch.substitutions.LaunchConfiguration('radial_distortion_coeffs'),
    ]

    calibration_node = launch_ros.actions.Node(package='camera_calibration',
                                               executable='cameracalibrator',
                                               arguments=calibration_args,
                                               remappings=[('image',
                                                            'image_offboard')])
    republish_node = launch_ros.actions.Node(package='image_transport',
                                             executable='republish',
                                             arguments=['compressed', 'raw'],
                                             remappings=[
                                                 ('in/compressed',
                                                  'image_raw/compressed'),
                                                 ('out', 'image_offboard'),
                                             ])

    nodes_group = launch.actions.GroupAction([
        launch_ros.actions.PushRosNamespace(
            launch.substitutions.LaunchConfiguration('vehicle_name')),
        launch_ros.actions.PushRosNamespace(
            launch.substitutions.LaunchConfiguration('camera_name')),
        republish_node, calibration_node
    ])

    return launch.LaunchDescription(launch_args + [
        nodes_group,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=calibration_node,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown()),
                ],
            )),
    ])
