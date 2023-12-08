from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    launch_description = LaunchDescription()

    component = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        namespace=LaunchConfiguration('camera_name'),
        name='rectifier',
        remappings=[
            ('image', 'image_raw'),
            ('camera_info', 'camera_info'),
            ('image_rect', 'image_rect'),
        ],
    )
    action = ComposableNodeContainer(
        name='image_proc_container',
        package='rclcpp_components',
        namespace='',
        executable='component_container',
        composable_node_descriptions=[component],
        output='screen',
    )
    launch_description.add_action(action)

    return launch_description
