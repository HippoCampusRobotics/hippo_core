from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    default_model_path = package_path / 'models/hippo3/urdf/hippo3.xacro'
    default_vehicle_name = 'uuv00'

    model_launch_arg = launch.actions.DeclareLaunchArgument(
        name='model_path',
        default_value=str(default_model_path),
        description='Absolute model path')
    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value=default_vehicle_name,
        description='Vehicle name used as namespace.')
    fake_estimator_launch_arg = launch.actions.DeclareLaunchArgument(
        name='fake_state_estimation', default_value='false')
    fake_vision_launch_arg = launch.actions.DeclareLaunchArgument(
        name='fake_vision', default_value='false')

    robot_description = launch.substitutions.LaunchConfiguration(
        'robot_description',
        default=launch.substitutions.Command([
            'ros2 run hippo_sim create_robot_description.py ', '--input ',
            launch.substitutions.LaunchConfiguration('model_path'),
            ' --mappings vehicle_name=',
            launch.substitutions.LaunchConfiguration('vehicle_name')
        ]))

    description = {'robot_description': robot_description}

    vehicle_group = launch.actions.GroupAction([
        launch_ros.actions.PushRosNamespace(
            launch.substitutions.LaunchConfiguration('vehicle_name')),
        launch_ros.actions.Node(package='hippo_sim',
                                executable='spawn',
                                parameters=[description],
                                arguments=[
                                    '--param',
                                    'robot_description',
                                    '--remove_on_exit',
                                    'true',
                                    '--x',
                                    '1.3',
                                    '--y',
                                    '1.0',
                                    '--z',
                                    '-0.5',
                                    '--Y',
                                    '1.5708',
                                ],
                                output='screen'),
        launch_ros.actions.Node(package='hippo_sim',
                                executable='bridge',
                                output='screen'),
        launch_ros.actions.Node(package='hippo_sim',
                                executable='fake_state_estimator',
                                name='state_estimator',
                                output='screen',
                                condition=launch.conditions.IfCondition(
                                    launch.substitutions.LaunchConfiguration(
                                        'fake_state_estimation'))),
        launch_ros.actions.Node(
            package='hippo_sim',
            executable="fake_vision",
            name="vision",
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('fake_vision'))),
        launch_ros.actions.Node(
            package='state_estimation',
            executable='estimator',
            name='state_estimator',
            condition=launch.conditions.UnlessCondition(
                launch.substitutions.LaunchConfiguration('fake_state_estimation'))),
    ])

    return launch.LaunchDescription([
        model_launch_arg,
        vehicle_name_launch_arg,
        fake_estimator_launch_arg,
        fake_vision_launch_arg,
        vehicle_group,
    ])
