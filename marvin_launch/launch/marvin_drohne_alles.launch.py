import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


MATCH_MODELS_SHARE = get_package_share_directory("marvin_models")


def generate_launch_description():
    default_px4_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../src/PX4-Autopilot'))
    rglgazebogui = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../src/RGLGazeboPlugin/install/RGLVisualize'))

    default_spawn_model = os.path.join(
        MATCH_MODELS_SHARE, "sdf", "marvin_drohne_alles", "model.sdf"
    )
    default_robot_description = os.path.join(
        MATCH_MODELS_SHARE, "sdf", "marvin_drohne_alles", "marvin_drohne_alles.xacro"
    )

    declare_args = [
        DeclareLaunchArgument(
            "world",
            default_value="scale",
            description="Gazebo-Weltname / PX4_GZ_WORLD.",
        ),
        DeclareLaunchArgument("spawn_x", default_value="-18.0", description="Spawn X."),
        DeclareLaunchArgument("spawn_y", default_value="0.0", description="Spawn Y."),
        DeclareLaunchArgument("spawn_z", default_value="1.0", description="Spawn Z."),
    ]

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
            "PX4_SIM_SPEED_FACTOR": "1",
        },
    )

    mavros_timer = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "launch", "mavros", "px4.launch",
                    "use_sim_time:=true",
                    "fcu_url:=udp://:14540@"
                ],
                output="screen"
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

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            default_robot_description,
        ]
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

        "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",

        "/rgl_lidar/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",

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
        + [px4_process, mavros_timer, robot_state_publisher_node, static_odometry_tf, static_lidar_tf, static_depth_tf, sensor_bridge_node, imu_timemachine]
    )
