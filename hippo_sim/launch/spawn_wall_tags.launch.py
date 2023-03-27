from ament_index_python.packages import get_package_share_path
import launch
import launch_ros
import math


def generate_launch_description():
    hippo_sim_path = get_package_share_path('hippo_sim')
    apriltag_path = str(hippo_sim_path / 'models/apriltag/urdf/apriltag.xacro')

    poses = [
        dict(xyz=(0.7, 4.0, -0.5), rpy=(math.pi / 2.0, 0.0, 0.0), tag_id=0),
        dict(xyz=(1.3, 4.0, -0.5), rpy=(math.pi / 2.0, 0.0, 0.0), tag_id=1),
        dict(xyz=(0.7, 4.0, -0.9), rpy=(math.pi / 2.0, 0.0, 0.0), tag_id=2),
        dict(xyz=(1.3, 4.0, -0.9), rpy=(math.pi / 2.0, 0.0, 0.0), tag_id=3),
    ]

    TAG_SIZE = 0.075
    nodes = []
    for i, p in enumerate(poses):
        description = launch.substitutions.LaunchConfiguration(
            f'descriptioin_{i}',
            default=launch.substitutions.Command([
                'ros2 run hippo_sim create_robot_description.py ',
                '--input ',
                apriltag_path,
                ' --mappings ',
                f'size_x={TAG_SIZE} size_y={TAG_SIZE} size_z=0.01 tag_id={p["tag_id"]}',
            ]))
        nodes.append(
            launch_ros.actions.Node(package='hippo_sim',
                                    executable='spawn',
                                    parameters=[{
                                        'robot_description': description
                                    }],
                                    arguments=[
                                        '--param',
                                        'robot_description',
                                        '--name',
                                        f'range_tag_{p["tag_id"]:02d}',
                                        '--x',
                                        f'{p["xyz"][0]}',
                                        '--y',
                                        f'{p["xyz"][1]}',
                                        '--z',
                                        f'{p["xyz"][2]}',
                                        '--R',
                                        f'{p["rpy"][0]}',
                                        '--P',
                                        f'{p["rpy"][1]}',
                                        '--Y',
                                        f'{p["rpy"][2]}',
                                    ],
                                    output='screen'))
    return launch.LaunchDescription(nodes)
