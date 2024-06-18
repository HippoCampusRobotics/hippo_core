from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from hippo_common.launch_helper import (
    LaunchArgsDict,
    config_file_path,
    declare_vehicle_name_and_sim_time,
)


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(launch_description=launch_description)

    pkg = 'hippo_control'
    config_file = config_file_path(pkg, 'actuator_mixer_bluerov_advanced.yaml')
    action = DeclareLaunchArgument('mixer_path', default_value=config_file)
    launch_description.add_action(action)

    default_path = config_file_path('esc', 'teensy_config.yaml')
    action = DeclareLaunchArgument(
        name='esc_config_file', default_value=default_path
    )
    launch_description.add_action(action)


def add_mixer_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='hippo_control',
        executable='actuator_mixer_bluerov_node',
        parameters=[
            args,
            LaunchConfiguration('mixer_path'),
        ],
        output='screen',
    )


def add_camera_node():
    action = Node(
        executable='v4l2_camera_node',
        package='v4l2_camera',
        name='front_camera',
        namespace='front_camera',
        parameters=[
            {
                'image_size': [640, 480],
            },
        ],
    )
    return action


def add_jpeg_camera_node():
    action = Node(
        executable='mjpeg_cam_node',
        package='mjpeg_cam',
        name='front_camera',
        namespace='front_camera',
        parameters=[
            {
                'device_id': 0,
                'discrete_size': 1,
                'fps': 30,
                'publish_nth_frame': 3,
            },
        ],
    )
    return action


def add_newton_gripper_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    action = Node(
        executable='newton_gripper_node',
        package='hardware',
        name='newton_gripper',
        parameters=[args],
    )
    return action


def add_esc_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    action = Node(
        executable='teensy_commander_node',
        package='esc',
        name='esc_commander',
        parameters=[args, LaunchConfiguration('esc_config_file')],
    )
    return action


def add_camera_servo_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    action = Node(
        executable='camera_servo_node',
        package='hardware',
        name='camera_servo',
        parameters=[args],
    )
    return action


def add_spotlight_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    action = Node(
        executable='spotlight_node',
        package='hardware',
        name='spotlight',
        parameters=[args],
    )
    return action


def add_micro_ros_agent():
    action = Node(
        executable='micro_ros_agent',
        package='micro_ros_agent',
        name='dds_agent',
        arguments=['serial', '--dev', '/dev/fcu_data', '-b', '921600'],
    )
    return action


def add_micro_xrce_agent():
    action = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent',
            'serial',
            '--dev',
            '/dev/fcu_data',
            '-b',
            '921600',
        ],
        output='screen',
    )
    return action


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)

    actions = [
        add_mixer_node(),
        # add_camera_node(),
        add_jpeg_camera_node(),
        add_newton_gripper_node(),
        add_camera_servo_node(),
        add_spotlight_node(),
        add_esc_node(),
    ]
    group = GroupAction(
        [PushRosNamespace(LaunchConfiguration('vehicle_name'))] + actions
    )
    launch_description.add_action(group)

    # launch_description.add_action(add_micro_ros_agent())
    launch_description.add_action(add_micro_xrce_agent())

    return launch_description
