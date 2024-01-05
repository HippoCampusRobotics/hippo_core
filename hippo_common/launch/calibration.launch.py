from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace

from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
)


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(
        launch_description=launch_description,
        use_sim_time_default="false",
    )
    action = DeclareLaunchArgument(
        name="compressed",
        description="Set to true if only a comrpessed image is pulished "
        "by the camera",
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(name='camera_name', )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name="grid_size",
        default_value="7x4",
        description="Grid size of inner corners of the calibration pattern.",
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name="square_size",
        default_value="0.03",
        description="Edge length of a single square [m].",
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name="camera_name",
        default_value="vertical_camera",
        description="The name of the camera.",
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name="vehicle_name",
        description="Vehicle name used as top level namespace.",
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name="radial_distortion_coeffs",
        default_value="3",
        description="Number of radial distortion coefficients. "
        "Between 2 and 6",
    )
    launch_description.add_action(action)


def include_image_decoder_node():
    package_path = get_package_share_path("mjpeg_cam")
    path = str(package_path / "launch/image_decoder.launch.py")
    source = PythonLaunchDescriptionSource(path)
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    image_decoder = IncludeLaunchDescription(
        source,
        condition=IfCondition(LaunchConfiguration('compressed')),
        launch_arguments=args.items(),
    )
    return image_decoder


def create_calibration_node():
    calibration_args = [
        "-s",
        LaunchConfiguration("grid_size"),
        "-q",
        LaunchConfiguration("square_size"),
        "-k",
        LaunchConfiguration("radial_distortion_coeffs"),
        "--no-service-check",
    ]
    return Node(
        package="camera_calibration",
        namespace=LaunchConfiguration('camera_name'),
        executable="cameracalibrator",
        arguments=calibration_args,
        remappings=[("image", "image_raw")],
        output="screen",
    )


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)

    nodes_group = GroupAction([
        PushROSNamespace(LaunchConfiguration("vehicle_name")),
        include_image_decoder_node(),
        create_calibration_node(),
    ])
    launch_description.add_action(nodes_group)

    return launch_description
