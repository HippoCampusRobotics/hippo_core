from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            name="grid_size",
            default_value="7x4",
            description="Grid size of inner corners of the calibration "
            "pattern.",
        ),
        DeclareLaunchArgument(
            name="square_size",
            default_value="0.03",
            description="Edge length of a single square [m].",
        ),
        DeclareLaunchArgument(
            name="camera_name",
            default_value="vertical_camera",
            description="The name of the camera.",
        ),
        DeclareLaunchArgument(
            name="vehicle_name",
            description="Vehicle name used as top level namespace.",
        ),
        DeclareLaunchArgument(
            name="radial_distortion_coeffs",
            default_value="3",
            description="Number of radial distortion coefficients. "
            "Between 2 and 6",
        ),
    ]

    calibration_args = [
        "-s",
        LaunchConfiguration("grid_size"),
        "-q",
        LaunchConfiguration("square_size"),
        "-k",
        LaunchConfiguration("radial_distortion_coeffs"),
        "--no-service-check",
    ]

    calibration_node = Node(
        package="camera_calibration",
        executable="cameracalibrator",
        arguments=calibration_args,
        remappings=[("image", "image_raw")],
        output="screen",
    )

    nodes_group = GroupAction([
        PushROSNamespace(LaunchConfiguration("vehicle_name")),
        PushROSNamespace(LaunchConfiguration("camera_name")),
        calibration_node,
    ])

    return LaunchDescription(launch_args + [nodes_group])
