"""Kompletter Nav-Stack auf FAST-LIO-Basis mit Persistenz + Relokalisierung —
in Simulation und auf der echten Drohne identisch, nur die Basis wechselt:

    ros2 launch marvin_launch nav_fastlio.launch.py            # sim:=true (Gazebo)
    ros2 launch marvin_launch nav_fastlio.launch.py sim:=false # echte Drohne

Frame-Kette (REP-105):
    map --(lidar_relocalization: GICP-Korrektur, init mit Montage-Pitch)--> camera_init
        --(FAST-LIO)--> body --(static, Lidar-Montage, von hier)--> base_link

FAST-LIO 'body' ist die IMU der MID360 — in Sim (RGL-Lidar-Link) und echt
gleich. Sie sitzt gekippt (Sim 0.52 rad aus model.sdf, echt ~31°), 10 cm vor /
7 cm ueber base_link. lidar_pitch/x/z levelt daraus TF (body->base_link), den
EKF2-Feed (odometry_to_drone) und map (Relokalisierungs-Init).
  Hebelarm im EKF2: auf der Drohne EKF2_EV_POS_X=0.10, _Y=0, _Z=-0.07 (FRD)
  setzen — odometry_to_drone liefert die Pose der IMU, nicht von base_link.

gps:=true: keine Lokalisierung (kein FAST-LIO/Relokalisierung) — PX4 fliegt
auf GPS, map->base_link kommt von mavros_local_to_tf. Fuer schnelle Tests
(Wegpunkte, Planer) ohne SLAM-Rechenlast. In der Sim schaltet das auch EKF2
um; auf der echten Drohne muss EKF2 dafuer per QGC auf GPS stehen.

Annahme: der PX4-Local-Frame deckt sich mit camera_init, weil die
FAST-LIO-Odometrie via odometry_to_drone in EKF2 gefuettert wird.
Deshalb laeuft hier KEIN mavros_local_to_tf — der wuerde map->base_link
doppelt claimen.

Workflow:
  1. Mapping-Flug:  ros2 launch marvin_launch nav_fastlio.launch.py map_pcd_save:=/pfad/karte.pcd map_bt:=/pfad/karte.bt
     - ohne map_pcd laeuft die Drift-Korrektur im Echtzeit-Modus:
       Referenz aus eingefrorenen Erstbesuchs-Keyframes (kein Prior noetig)
     - am Ende beide Karten speichern:
       ros2 service call /lidar_relocalization/save_map std_srvs/srv/Trigger
       ros2 service call /nav_node/save_map std_srvs/srv/Trigger
       (NICHT FAST-LIOs PCD nehmen — die liegt in camera_init inkl. Drift.)
  2. Folgeflug:     ... map_pcd:=/pfad/karte.pcd map_bt:=/pfad/karte.bt
     - Relokalisierung matcht gegen die PCD-Karte, Planer startet mit Prior
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Lidar-(=FAST-LIO-body-)Montage relativ zu base_link auf der echten Drohne.
# ponytail: grob gemessen — Kalibrier-Knopf sind die Launch-Args lidar_pitch/x/z.
# Check: Drohne level hinstellen, in RViz muss der Boden in 'map' waagrecht liegen.
REAL_MOUNT = {'lidar_pitch': math.radians(31.0), 'lidar_x': 0.10, 'lidar_z': 0.07}
# Sim: Mount der MID360 aus marvin_drohne_alles/model.sdf (0.1 0 0.07 0 0.52 0)
SIM_MOUNT = {'lidar_pitch': 0.52, 'lidar_x': 0.10, 'lidar_z': 0.07}

# Pro Modus: Startverzoegerungen [s] und OctoMap-Quellen. Lidar = Hauptquelle
# (/cloud_registered liegt in camera_init -> Strahl-Ursprung 'body'), Kamera
# nur Nahbereich (3 m, nav_node camera_max_range_). Frame leer = aus dem Header.
SIM = {'fastlio_delay': 30.0, 'nav_delay': 35.0,
       'camera_topic': '/marvin_drohne_alles/front_depth/image/points',
       'camera_frame': 'front_depth_camera_optical_frame'}
REAL = {'fastlio_delay': 5.0, 'nav_delay': 10.0,
        'camera_topic': '/camera/camera/depth/color/points',
        'camera_frame': ''}
LIDAR_FASTLIO = ('/cloud_registered', 'body')
# Ohne FAST-LIO: Roh-Lidar im Sensor-Frame. Echt kommt die MID360 als
# Livox-CustomMsg (kein PointCloud2) -> dort nur Kamera.
LIDAR_GPS = {True: ('/rgl_lidar', ''), False: ('', '')}


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    sim = arg('sim').lower() in ('true', '1')
    gps = arg('gps').lower() in ('true', '1')
    mode = SIM if sim else REAL
    launch_dir = os.path.join(get_package_share_directory('marvin_launch'), 'launch')

    # FAST-LIO laeuft in Sim und echt auf der Lidar-IMU -> Montage je Modus; Args ueberschreiben
    mount = {k: (float(arg(k)) if arg(k) else v) for k, v in (SIM_MOUNT if sim else REAL_MOUNT).items()}
    pitch = mount['lidar_pitch']

    if sim:
        base = [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'world2.launch.py')),
                launch_arguments={'world': arg('world')}.items()),
            # startet PX4 + MAVROS + Bridges; body->base_link kommt von hier (Montage)
            TimerAction(period=15.0, actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'marvin_drohne_alles.launch.py')),
                launch_arguments={'gps': arg('ekf_gps') or str(gps).lower(), 'world': arg('world'),
                                  'body_tf': 'false',
                                  **{k: arg(k) for k in ('spawn_x', 'spawn_y', 'spawn_z')}}.items())]),
        ]
    else:
        base = [IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'marvin_real_base.launch.py')))]
    if not gps:
        # body (= Lidar-IMU) -> base_link = Inverse der Lidar-Pose in base_link
        # (R=Ry(pitch), t): R^T = Ry(-pitch), Translation -R^T t
        c, s = math.cos(pitch), math.sin(pitch)
        tx, tz = mount['lidar_x'], mount['lidar_z']
        bx, bz = -(c * tx - s * tz), -(s * tx + c * tz)
        base.append(Node(package='tf2_ros', executable='static_transform_publisher',
                         name='body_to_base_link',
                         parameters=[{'use_sim_time': sim}],
                         arguments=['--x', str(bx), '--z', str(bz), '--pitch', str(-pitch),
                                    '--frame-id', 'body', '--child-frame-id', 'base_link']))

    use_sim_time = {'use_sim_time': sim}
    lidar_topic, lidar_frame = LIDAR_GPS[sim] if gps else LIDAR_FASTLIO

    fastlio = TimerAction(period=mode['fastlio_delay'], actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')),
        launch_arguments={
            'config_path': os.path.join(get_package_share_directory('marvin_launch'), 'config'),
            'config_file': 'fastlio_sim.yaml' if sim else 'fastlio_real.yaml',
            'use_sim_time': str(sim).lower(),
            # Bordrechner hat kein Display; in der Sim abschaltbar (rviz:=false spart ~1 Kern, Web-UI zeigt dasselbe)
            'rviz': arg('rviz') or str(sim).lower(),
            # Fixed Frame map — camera_init ist die Lage der gekippten Lidar-IMU
            'rviz_cfg': os.path.join(get_package_share_directory('marvin_launch'), 'config', 'nav.rviz'),
        }.items())])

    # FAST-LIO-Odometrie -> PX4 EKF2 (vision odometry), gelevelt um die Montage
    odom_to_drone = Node(
        package='marvin_utils',
        executable='odometry_to_drone',
        name='odometry_to_drone',
        output='screen',
        parameters=[{**use_sim_time, 'mount_pitch': pitch}],
    )

    # map -> camera_init: GICP-Korrektur. Mit map_pcd gegen die gespeicherte
    # Karte (Relokalisierung), ohne map_pcd Echtzeit-Modus — Referenz aus
    # eingefrorenen Erstbesuchs-Keyframes, deckelt FAST-LIO-Drift im Flug.
    # initial_pitch levelt map gegenueber dem gekippten camera_init.
    relocalization = Node(
        package='marvin_nav',
        executable='lidar_relocalization_node',
        name='lidar_relocalization',
        output='screen',
        parameters=[{
            **use_sim_time,
            'map_pcd': arg('map_pcd'),
            'map_save_path': arg('map_pcd_save'),
            'initial_x': float(arg('initial_x')),
            'initial_y': float(arg('initial_y')),
            'initial_z': float(arg('initial_z')),
            'initial_yaw': float(arg('initial_yaw')),
            'initial_pitch': pitch,
        }],
    )

    # Mapping (in-process OctoMap) + globaler/lokaler Planer
    nav_node = TimerAction(period=mode['nav_delay'], actions=[Node(
        package='marvin_nav',
        executable='mapper_planner_node',
        name='nav_node',
        output='screen',
        parameters=[{
            **use_sim_time,
            'cloud_topic': lidar_topic,
            'sensor_frame': lidar_frame,
            'camera_topic': mode['camera_topic'],
            'camera_frame': mode['camera_frame'],
            'map_file': arg('map_bt'),
        }],
    )])

    # Pursuit trackt in map (Pose via TF map->base_link wie der Planer)
    # und dreht Kommandos selbst in den PX4-Frame
    pursuit = Node(
        package='marvin_utils',
        executable='pursuit',
        name='pure_pursuit_tracker',
        output='screen',
        # lookahead = Carrot-Abstand; PX4s Positionsregler macht daraus ~die Reisegeschwindigkeit
        parameters=[{**use_sim_time, 'lookahead_dist': float(arg('lookahead'))}],
    )

    if gps:
        # map->base_link direkt aus der PX4-Pose (ohne FAST-LIO gibt es kein 'body')
        pose_tf = Node(
            package='marvin_utils',
            executable='mavros_local_to_tf',
            name='mavros_local_to_tf',
            output='screen',
            parameters=[use_sim_time],
        )
        return base + [pose_tf, nav_node, pursuit]
    nodes = base + [fastlio, odom_to_drone, relocalization, nav_node, pursuit]
    if arg('keyframe_dir'):
        # Mapping-Modus: Keyframes fuer die Pose-Graph-Optimierung (map_optimizer)
        nodes.append(Node(
            package='marvin_utils',
            executable='keyframe_recorder',
            name='keyframe_recorder',
            output='screen',
            parameters=[{**use_sim_time, 'out_dir': arg('keyframe_dir'), 'mount_pitch': pitch}],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true',
                              description='true = Gazebo/PX4-SITL, false = echte Drohne'),
        DeclareLaunchArgument('rviz', default_value='', description='leer = in der Sim an, auf der Drohne aus'),
        DeclareLaunchArgument('gps', default_value='false',
                              description='true = ohne FAST-LIO/Relokalisierung, PX4 fliegt auf GPS'),
        DeclareLaunchArgument('world', default_value='scale3', description='Sim: marvin_models/worlds/<world>.sdf'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='2.0'),
        DeclareLaunchArgument('ekf_gps', default_value='',
                              description='Sim: EKF2-Quelle getrennt waehlen; true = PX4 auf GPS, FAST-LIO laeuft nur mit (Drift-Tests)'),
        DeclareLaunchArgument('lookahead', default_value='1.0',
                              description='pursuit: Carrot-Abstand [m] (~Reisegeschwindigkeit in m/s)'),
        DeclareLaunchArgument('keyframe_dir', default_value='',
                              description='Mapping: Keyframes fuer map_optimizer hierhin schreiben (mapping.launch.py setzt das)'),
        DeclareLaunchArgument('map_pcd', default_value='',
                              description='Gespeicherte Punktwolkenkarte (.pcd) fuer Relokalisierung; leer = Mapping-Modus'),
        DeclareLaunchArgument('map_pcd_save', default_value='',
                              description='Ziel (.pcd) fuer /lidar_relocalization/save_map'),
        DeclareLaunchArgument('map_bt', default_value='',
                              description='OctoMap-Datei (.bt): laden beim Start / Ziel fuer ~/save_map'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_z', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        # leer = Default je Modus (SIM_MOUNT / REAL_MOUNT)
        DeclareLaunchArgument('lidar_pitch', default_value='', description='Lidar-Montage-Pitch [rad]'),
        DeclareLaunchArgument('lidar_x', default_value='', description='Lidar vor base_link [m]'),
        DeclareLaunchArgument('lidar_z', default_value='', description='Lidar ueber base_link [m] (unter = negativ)'),
        OpaqueFunction(function=launch_setup),
    ])
