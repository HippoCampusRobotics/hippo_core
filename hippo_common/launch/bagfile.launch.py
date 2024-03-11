import launch


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=[
                'ros2',
                'bag',
                'record',
                '-o',
                'onboard_lemniscate_v_0.4',
                '/uuv02/odometry',
                '/uuv02/ground_truth/odometry',
                '/uuv02/odometry_naive',
                '/uuv02/vertical_camera/image_raw/compressed',
                '/uuv02/vertical_camera/camera_info',
                '/uuv02/event_camera/events',
                '/uuv02/event_camera/imu',
            ],
            output='screen',
        )
    ])
