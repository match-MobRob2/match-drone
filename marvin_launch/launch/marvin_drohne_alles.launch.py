import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


MATCH_MODELS_SHARE = get_package_share_directory("marvin_models")


def generate_launch_description():
    default_px4_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../src/PX4-Autopilot'))
    rglgazebogui = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../src/RGLGazeboPlugin/install/RGLVisualize'))

    default_spawn_model = os.path.join(
        MATCH_MODELS_SHARE, "sdf", "marvin_drohne_alles", "model.sdf"
    )
    # default_robot_description = os.path.join(
    #     MATCH_MODELS_SHARE, "sdf", "marvin_drohne_alles", "marvin_drohne_alles.xacro"
    # )
    default_robot_description = os.path.join(
        MATCH_MODELS_SHARE, "sdf", "marvin_drohne_alles", "marvin_drohne_alles.xacro"
    )

    declare_args = [
        DeclareLaunchArgument(
            "world",
            default_value="scale3",
            description="Gazebo-Weltname / PX4_GZ_WORLD.",
        ),
        DeclareLaunchArgument("spawn_x", default_value="0.0", description="Spawn X."),
        DeclareLaunchArgument("spawn_y", default_value="0.0", description="Spawn Y."),
        DeclareLaunchArgument("spawn_z", default_value="2.0", description="Spawn Z."),
        DeclareLaunchArgument("body_tf", default_value="true",
                              description="Identitaets-TF body->base_link (FAST-LIO auf base_link-IMU)"),
        DeclareLaunchArgument("gps", default_value="true",
                              description="true: EKF2 auf GPS; false: EKF2 auf Vision-Odometrie (FAST-LIO, wie echt)"),
    ]

    # EKF2-Quelle per PX4_PARAM_*-Env (rcS setzt sie beim Start). Beide Modi
    # explizit, weil gesetzte Params in parameters.bson ueberleben.
    def ekf(if_gps, if_vision):
        return PythonExpression(["'", if_gps, "' if '", LaunchConfiguration("gps"),
                                 "'.lower() in ('true', '1') else '", if_vision, "'"])

    world = LaunchConfiguration("world")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")

    px4_process = ExecuteProcess(
        cmd=["./build/px4_sitl_default/bin/px4"],
        cwd=default_px4_dir,
        output="screen",
        additional_env={
            "PX4_SYS_AUTOSTART": "40014",
            "PX4_SIM_MODEL": "marvin_drohne_alles",
            "PX4_SIMULATOR": "GZ",
            "PX4_GZ_MODEL_POSE": [
                spawn_x,
                TextSubstitution(text=","),
                spawn_y,
                TextSubstitution(text=","),
                spawn_z,
                TextSubstitution(text=",0,0,0"),
            ],
            "PX4_GZ_STANDALONE": "1",
            "PX4_GZ_WORLD": world,
            "PX4_HOME_LAT": "52.42449457140792",
            "PX4_HOME_LON": "9.620245153463955",
            "PX4_HOME_ALT": "20.0",
            "PX4_PARAM_EKF2_GPS_CTRL": ekf("7", "0"),
            # Vision: Pos hor+vert + Yaw (11), Hoehen-Referenz Vision (3), Baro fusioniert
            # weiter als Stuetze (BARO_CTRL-Default 1). Baro-Referenz getestet: Sim-Baro zu
            # traege -> 3 m Ueberschwinger beim Takeoff. Schutz gegen FAST-LIO-Divergenz:
            # Plausibilitaets-Gate in odometry_to_drone.
            "PX4_PARAM_EKF2_EV_CTRL": ekf("0", "11"),
            "PX4_PARAM_EKF2_HGT_REF": ekf("1", "3"),    # 1 = GPS, 3 = Vision
            "PX4_PARAM_EKF2_MAG_TYPE": ekf("0", "5"),   # Vision-Yaw statt Kompass
        },
    )

    # mavros_node direkt statt px4.launch: das XML reicht use_sim_time nicht
    # durch -> MAVROS stempelte mit Wanduhr, EKF2 verwarf die (sim-gestempelte)
    # Vision-Odometrie als veraltet
    mavros_share = get_package_share_directory("mavros")
    mavros_timer = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="mavros",
                executable="mavros_node",
                namespace="mavros",
                output="screen",
                parameters=[
                    os.path.join(mavros_share, "launch", "px4_pluginlists.yaml"),
                    os.path.join(mavros_share, "launch", "px4_config.yaml"),
                    {"fcu_url": "udp://:14540@", "gcs_url": "", "tgt_system": 1,
                     "tgt_component": 1, "fcu_protocol": "v2.0", "use_sim_time": True},
                ],
            )
        ],
    )

    def make_spawn_timer(context, *args, **kwargs):
        spawn_process = ExecuteProcess(
            additional_env={
                "GZ_GUI_PLUGIN_PATH": rglgazebogui,
            },
            cmd=[
                "ros2",
                "run",
                "ros_gz_sim",
                "create",
                "-world",
                world,
                "-name",
                "marvin_drohne_alles",
                "-file",
                default_spawn_model,
                "-x",
                spawn_x,
                "-y",
                spawn_y,
                "-z",
                spawn_z,
                "-R",
                "0.0",
                "-P",
                "0.0",
                "-Y",
                "0.0",
            ],
            output="screen",
        )

        return [TimerAction(period=3.0, actions=[spawn_process])]

    spawn_action = OpaqueFunction(function=make_spawn_timer)

    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                default_robot_description,
            ]
        ),
        value_type=str,
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": True}],
    )

    static_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='top_lidar_sensor_frame_tf',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'top_lidar_frame',
            'marvin_drohne_alles_0/top_lidar_link/top_lidar',
        ],
        parameters=[{"use_sim_time": True}],
    )

    static_odometry_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odometry_frame_tf',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'body',
            'base_link',
        ],
        parameters=[{"use_sim_time": True}],
        # nav_fastlio publiziert body->base_link selbst (Lidar-IMU-Montage)
        condition=IfCondition(LaunchConfiguration("body_tf")),
    )


    # Sim-MID360 (RGL) publiziert in 'RGLLidar'; Montage aus model.sdf
    static_rgl_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rgl_lidar_frame_tf',
        arguments=['--x', '0.1', '--z', '0.07', '--pitch', '0.52',
                   '--frame-id', 'base_link', '--child-frame-id', 'RGLLidar'],
        parameters=[{"use_sim_time": True}],
    )

    static_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='front_depth_sensor_frame_tf',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'front_depth_camera_optical_frame',
            'marvin_drohne_alles_0/front_sensor_mount_link/front_depth_camera',
        ],
        parameters=[{"use_sim_time": True}],
    )

    bridge_arguments = [
        "/marvin_drohne_alles/front_rgb/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/marvin_drohne_alles/front_rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",

        "/marvin_drohne_alles/front_depth/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/marvin_drohne_alles/front_depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/marvin_drohne_alles/front_depth/image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",

        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

        "/rgl_lidar/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",

        # base_link-IMU (Sim-Zeitstempel, 200 Hz) fuer FAST-LIO — ersetzt die
        # re-gestempelte MAVROS-IMU (Arrival-Time-Jitter drehte die Karte bei Yaw)
        "/marvin_drohne/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",

        "/rgl_lidar@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
    ]


    sensor_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_arguments,
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    mavros_local_to_tf_node = Node(
        package="marvin_utils",
        executable="mavros_local_to_tf",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    imu_timemachine = Node(
        package="marvin_utils",
        executable="imu_timemachine",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    cloud_filter_node = Node(
        package="marvin_utils",
        executable="cloud_nan_filter",
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    pointcloud_to_livox = Node(
        package="marvin_utils",
        executable="pointcloud_to_livox",
        parameters=[{"use_sim_time": True}],
        output="screen"
    )


    return LaunchDescription(
        declare_args
        + [px4_process, mavros_timer, robot_state_publisher_node, static_odometry_tf, static_lidar_tf, static_rgl_tf, static_depth_tf, sensor_bridge_node, imu_timemachine]
    )
