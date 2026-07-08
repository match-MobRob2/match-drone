"""Kompletter Nav-Stack auf FAST-LIO-Basis mit Persistenz + Relokalisierung.

Frame-Kette (REP-105):
    map --(lidar_relocalization: GICP-Korrektur)--> camera_init
        --(FAST-LIO)--> body --(static, aus marvin_drohne_alles.launch)--> base_link

Annahme: der PX4-Local-Frame deckt sich mit camera_init, weil die
FAST-LIO-Odometrie via odometry_to_drone in EKF2 gefuettert wird.
Deshalb laeuft hier KEIN mavros_local_to_tf — der wuerde map->base_link
doppelt claimen.

Workflow:
  1. Mapping-Flug:  ros2 launch marvin_launch nav_fastlio.launch.py
     - ohne map_pcd laeuft die Drift-Korrektur im Echtzeit-Modus:
       Referenz aus eingefrorenen Erstbesuchs-Keyframes (kein Prior noetig)
     - FAST-LIO speichert die Punktwolkenkarte beim Beenden als PCD
       (pcd_save_en in der FAST-LIO-Config)
     - OctoMap speichern: ros2 service call /nav_node/save_map std_srvs/srv/Trigger
       (Ziel-Datei via map_bt:=/pfad/karte.bt)
  2. Folgeflug:     ... map_pcd:=/pfad/scans.pcd map_bt:=/pfad/karte.bt
     - Relokalisierung matcht gegen die PCD-Karte, Planer startet mit Prior
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_pcd = LaunchConfiguration('map_pcd')
    map_bt = LaunchConfiguration('map_bt')

    declare_args = [
        DeclareLaunchArgument('map_pcd', default_value='',
                              description='Gespeicherte Punktwolkenkarte (.pcd) fuer Relokalisierung; leer = Mapping-Modus'),
        DeclareLaunchArgument('map_bt', default_value='',
                              description='OctoMap-Datei (.bt): laden beim Start / Ziel fuer ~/save_map'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_z', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
    ]

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('marvin_launch'),
                         'launch', 'world2.launch.py')
        )
    )

    drone_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('marvin_launch'),
                                 'launch', 'marvin_drohne_alles.launch.py')
                )
            )
        ]
    )

    fastlio = TimerAction(
        period=30.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('fast_lio'),
                                 'launch', 'mapping.launch.py')
                ),
                launch_arguments={
                    'config_file': '/mnt/daten/match_drone/src/FAST_LIO_ROS2/config/mid360.yaml'
                }.items()
            )
        ]
    )

    # FAST-LIO-Odometrie -> PX4 EKF2 (vision odometry)
    odom_to_drone = Node(
        package='marvin_utils',
        executable='odometry_to_drone',
        name='odometry_to_drone',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # map -> camera_init: GICP-Korrektur. Mit map_pcd gegen die gespeicherte
    # Karte (Relokalisierung), ohne map_pcd Echtzeit-Modus — Referenz aus
    # eingefrorenen Erstbesuchs-Keyframes, deckelt FAST-LIO-Drift im Flug.
    relocalization = Node(
        package='marvin_nav',
        executable='lidar_relocalization_node',
        name='lidar_relocalization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_pcd': map_pcd,
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_z': LaunchConfiguration('initial_z'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
        }],
    )

    # Mapping (in-process OctoMap) + globaler/lokaler Planer.
    # cloud_registered ist vor-registriert in camera_init -> sensor_frame=body
    # liefert den echten Sensor-Ursprung fuers Raycasting.
    nav_node = TimerAction(
        period=35.0,
        actions=[
            Node(
                package='marvin_nav',
                executable='mapper_planner_node',
                name='nav_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'cloud_topic': '/marvin_drohne_alles/front_depth/image/points',
                    'sensor_frame': 'front_depth_camera_optical_frame',
                    'map_file': map_bt,
                }],
            )
        ]
    )

    pursuit = Node(
        package='marvin_utils',
        executable='pursuit',
        name='pure_pursuit_tracker',
        output='screen',
        parameters=[{
            # Pursuit trackt in map (Pose via TF map->base_link wie der Planer)
            # und dreht Kommandos selbst in den PX4-Frame — kein control_frame mehr
            'use_sim_time': True,
        }],
    )

    return LaunchDescription(declare_args + [
        world_launch,
        drone_launch,
        fastlio,
        odom_to_drone,
        relocalization,
        nav_node,
        pursuit,
    ])
