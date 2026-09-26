"""Echte Drohne: Sensoren, MAVROS, LEDs, Recorder — Gegenstueck zu
world2.launch.py + marvin_drohne_alles.launch.py in der Simulation.

Allein nutzbar (z.B. nur Sensoren + Bag aufnehmen) oder via
    ros2 launch marvin_launch nav_fastlio.launch.py sim:=false

Erwartet udev-Symlinks /dev/cube_orange und /dev/arduino (per Arg aenderbar).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_args = [
        DeclareLaunchArgument('fcu_url', default_value='/dev/cube_orange:57600'),
        DeclareLaunchArgument('led_port', default_value='/dev/arduino'),
        DeclareLaunchArgument('bag_dir', default_value='~/flight_logs'),
    ]

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('mavros'), 'launch', 'px4.launch'])),
        launch_arguments={'fcu_url': LaunchConfiguration('fcu_url')}.items(),
    )

    livox = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,      # CustomMsg -> FAST-LIO lidar_type 1
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            # IP-Konfiguration der MID360 steht in dieser JSON
            'user_config_path': os.path.join(
                get_package_share_directory('livox_ros_driver2'), 'config', 'MID360_config.json'),
            'cmdline_input_bd_code': 'livox0000000001',
        }],
    )

    realsense = TimerAction(period=8.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'enable_gyro': 'false',
            'enable_accel': 'false',
        }.items(),
    )])

    # ponytail: Kamera-Montage aus dem Sim-Modell (realsense_link in
    # marvin_drohne_alles/model.sdf) uebernommen — an der echten Drohne nachmessen.
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.1', '--z', '-0.08', '--pitch', '0.2',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
    )

    light_controller = Node(
        package='marvin_utils',
        executable='light_controller',
        name='light_controller',
        output='screen',
        parameters=[{'port': LaunchConfiguration('led_port')}],
    )

    light_watchdog = Node(
        package='marvin_utils',
        executable='light_watchdog',
        name='light_watchdog',
        output='screen',
    )

    # Startet ros2 bag record beim Armen, stoppt beim Disarmen
    recorder = TimerAction(period=15.0, actions=[Node(
        package='marvin_utils',
        executable='recorder',
        name='rosbag_recorder_node',
        output='screen',
        parameters=[{'output_dir': LaunchConfiguration('bag_dir')}],
    )])

    return LaunchDescription(declare_args + [
        mavros,
        livox,
        realsense,
        camera_tf,
        light_controller,
        light_watchdog,
        recorder,
    ])
