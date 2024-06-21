# Copyright (C) 2023-2024 Thies Lennart Alff
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
# USA

from typing import Iterable

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def config_file_path(pkg_name: str, file_name: str):
    path = get_package_share_path(pkg_name) / 'config' / file_name
    return str(path)


def launch_file_source(pkg_name: str, file_name: str):
    path = str(get_package_share_path(pkg_name) / 'launch' / file_name)
    source = PythonLaunchDescriptionSource(path)
    return source


def validate_vehicle_type(context, valid_vehicles, *args, **kwargs):
    value = LaunchConfiguration('vehicle_type').perform(context)
    if value not in valid_vehicles:
        raise ValueError(f'"Any of vehicle_type:={valid_vehicles}" is required')


def require_hippocampus_vehicle_type() -> Action:
    return OpaqueFunction(
        function=validate_vehicle_type,
        args=[['hippocampus']],
    )


def require_bluerov_vehicle_type() -> Action:
    return OpaqueFunction(function=validate_vehicle_type, args=[['bluerov']])


def required_valid_vehicle_type() -> Action:
    return OpaqueFunction(
        function=validate_vehicle_type,
        args=[['hippocampus', 'bluerov']],
    )


class LaunchArgsDict(dict):
    def add(self, name: str | Iterable[str]) -> None:
        """Add a LaunchConfiguration as a key value pair to the internal dict.

        Args:
            name: Name(s) of the launch configuration.
        """
        if isinstance(name, str):
            super().__setitem__(name, LaunchConfiguration(name))
        else:
            for n in name:
                super().__setitem__(n, LaunchConfiguration(n))

    def add_sim_time(self):
        super().__setitem__('use_sim_time', LaunchConfiguration('use_sim_time'))

    def add_vehicle_name(self):
        super().__setitem__('vehicle_name', LaunchConfiguration('vehicle_name'))

    def add_vehicle_name_and_sim_time(self):
        self.add_sim_time()
        self.add_vehicle_name()

    def add_vehicle_type(self):
        super().__setitem__('vehicle_type', LaunchConfiguration('vehicle_type'))


def declare_vehicle_name_and_sim_time(
    launch_description: LaunchDescription, use_sim_time_default=None
):
    declare_vehicle_name(launch_description=launch_description)
    declare_use_sim_time(
        launch_description=launch_description, default=use_sim_time_default
    )


def declare_vehicle_type(launch_description: LaunchDescription):
    action = DeclareLaunchArgument(
        'vehicle_type',
        description='Either "hippocampus" or "bluerov".',
    )
    launch_description.add_action(action)


def declare_vehicle_name(launch_description: LaunchDescription):
    action = DeclareLaunchArgument(
        'vehicle_name', description='Vehicle name used as namespace.'
    )
    launch_description.add_action(action)


def declare_use_sim_time(launch_description: LaunchDescription, default=None):
    action = DeclareLaunchArgument(
        'use_sim_time',
        description=(
            'Decides wether to use the wall time or the clock topic '
            'as time reference. Set to TRUE for simulation.'
        ),
        default_value=default,
    )
    launch_description.add_action(action)


def create_camera_bridge(
    vehicle_name: str | Substitution,
    camera_name: str | Substitution,
    use_camera: LaunchConfiguration,
    image_name: str | Substitution = 'image_rect',
) -> GroupAction:
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

    node_group = GroupAction(
        launch_configurations=group_configs,
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
                condition=IfCondition(use_camera),
                output='screen',
            ),
        ],
    )

    return node_group
