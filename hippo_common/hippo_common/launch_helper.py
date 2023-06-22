from launch.substitution import Substitution
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def create_camera_bridge(
        vehicle_name: str | Substitution,
        camera_name: str | Substitution,
        image_name: str | Substitution = 'image_rect') -> GroupAction:
    base_topic = PathJoinSubstitution(['/', vehicle_name, camera_name])
    image_topic = PathJoinSubstitution([base_topic, image_name])
    image_types = '@sensor_msgs/msg/Image[ignition.msgs.Image'
    camera_info_topic = PathJoinSubstitution([base_topic, 'camera_info'])
    camera_info_types = '@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'

    group_configs = {
        'camera_info': [camera_info_topic, camera_info_types],
        'image_topic': [image_topic, image_types],
        # 'image_topic': [image_topic],
        'name': [camera_name],
    }

    node_group = GroupAction(launch_configurations=group_configs,
                             actions=[
                                 PushRosNamespace(vehicle_name),
                                 Node(
                                     package='ros_gz_bridge',
                                     executable='parameter_bridge',
                                     name=LaunchConfiguration('name'),
                                     arguments=[
                                         LaunchConfiguration('camera_info'),
                                         LaunchConfiguration('image_topic'),
                                     ],
                                     output='screen',
                                 ),
                             ])

    return node_group
