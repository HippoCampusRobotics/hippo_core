from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_path = get_package_share_path('hippo_sim')
    default_urdf_path = package_path / 'models/apriltag/urdf/apriltag.xacro'

    launch_args = [
        launch.actions.DeclareLaunchArgument(
            name='urdf_path',
            default_value=str(default_urdf_path),
            description='Absolute path to apriltag .xacro'),
        launch.actions.DeclareLaunchArgument(
            name='tag_rows',
            default_value='13',
            description='Number of apriltag rows.'),
        launch.actions.DeclareLaunchArgument(
            name='tag_cols',
            default_value='7',
            description='Number of apriltag columns.'),
        launch.actions.DeclareLaunchArgument(
            name='tag_distance_x',
            default_value='0.25',
            description='Distance between tags in meters.'),
        launch.actions.DeclareLaunchArgument(
            name='tag_distance_y',
            default_value='0.3',
            description='Distance between tags in meters.'),
        launch.actions.DeclareLaunchArgument(
            name='tag_offset_x',
            default_value='0.2',
            description=('Offset of the first tag from the origin of the world '
                         'coordinate system.')),
        launch.actions.DeclareLaunchArgument(
            name='tag_offset_y',
            default_value='0.3',
            description=('Offset of the first tag from the origin of the world '
                         'coordinate system.')),
        launch.actions.DeclareLaunchArgument(
            name='tag_offset_z',
            default_value='-1.45',
            description=('Offset of the first tag from the origin of the world '
                         'coordinate system.')),
        launch.actions.DeclareLaunchArgument(
            name='tag_size',
            default_value='0.05',
            description='Tag size including the white border (i.e. 10x10 px)'),
        launch.actions.DeclareLaunchArgument(
            name='tag_thickness',
            default_value='0.01',
            description='Thickness of the tag model.'),
    ]

    tag_size = [
        ' --tag-size ',
        launch.substitutions.LaunchConfiguration('tag_size'),
    ]
    grid_size = [
        ' --grid-size ',
        launch.substitutions.LaunchConfiguration('tag_rows'),
        ' ',
        launch.substitutions.LaunchConfiguration('tag_cols'),
    ]
    offset = [
        ' --offset ',
        launch.substitutions.LaunchConfiguration('tag_offset_x'),
        ' ',
        launch.substitutions.LaunchConfiguration('tag_offset_y'),
        ' ',
        launch.substitutions.LaunchConfiguration('tag_offset_z'),
    ]
    distance = [
        ' --distance ',
        launch.substitutions.LaunchConfiguration('tag_distance_x'),
        ' ',
        launch.substitutions.LaunchConfiguration('tag_distance_y'),
    ]
    out_dir = [' --out-dir -']

    tag_poses = launch.substitutions.LaunchConfiguration(
        'tag_poses',
        default=launch.substitutions.Command([
            'ros2 run hippo_sim generate_tag_poses.py',
        ] + tag_size + grid_size + offset + distance + out_dir))

    tag_poses_arg = [' --tag-poses \'', tag_poses, '\'']
    sdf = launch.substitutions.LaunchConfiguration(
        'sdf',
        default=launch.substitutions.Command([
            'ros2 run hippo_sim generate_pool.py',
        ] + tag_poses_arg))
    params = {'description': sdf}

    return launch.LaunchDescription(launch_args + [
        launch_ros.actions.Node(package='hippo_sim',
                                executable='spawn',
                                parameters=[params],
                                arguments=['--param', 'description'],
                                output='screen')
    ])
