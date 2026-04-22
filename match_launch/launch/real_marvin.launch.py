# realsense.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'px4.launch'
            ])
        ]),
        launch_arguments={
            'fcu_url': '/dev/cube_orange:57600'
        }.items()
    )

    imu_timemachine = TimerAction(
        period=5.0,
        actions=[Node(
            package='match_utils',
            executable='imu_timemachine',
            output='screen'
        )]
    )

    realsense = TimerAction(
            period=8.0,
            actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ]),
            launch_arguments={
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
                'pointcloud.enable': 'true',
                'align_depth.enable': 'true',
                'enable_gyro': 'false',
                'enable_accel': 'false',
            }.items()
        )]
        )

    livox = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {"xfer_format": 1},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": 'livox_frame'},
            {"lvx_file_path": '/home/livox/livox_test.lvx'},
            {"user_config_path": '/home/marvin-drone/match-drone/src/livox_ros_driver2/config/MID360_config.json'},
            {"cmdline_input_bd_code": 'livox0000000001'}
        ]
    )

    tf_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
            output='screen'
        )

    fast_lio = TimerAction(
        period=12.0,  # nach mavros (5s) + imu_timemachine
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('fast_lio'),
                    'launch',
                    'mapping.launch.py'
                )
            ),
            launch_arguments={
                'config_file': '/home/marvin-drone/match-drone/src/FAST_LIO_ROS2/config/mid360.yaml'
            }.items()
        )]
    )

    light_controller = Node(
        package='match_utils',
        executable='light_controller',
        name='light_controller',
        output='screen',
    )

    light_watchdog = Node(
        package='match_utils',
        executable='light_watchdog',
        name='light_watchdog',
        output='screen',
    )

    recorder = TimerAction(
        period=15.0,  # nach mavros (5s) + imu_tim
        actions=[Node(
            package='match_utils',
            executable='recorder',
            name='rosbag_recorder_node',
            output='screen',
        )]
    )

    return LaunchDescription([
        mavros,
        imu_timemachine,
        livox,
        realsense,
        light_controller,
        light_watchdog,
        recorder
    ])