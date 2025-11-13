import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


MATCH_MODELS_SHARE = get_package_share_directory("match_models")


def generate_launch_description():
    default_px4_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "PX4-Autopilot")
    )
    default_spawn_model = os.path.join(
        MATCH_MODELS_SHARE, "sdf", "match_drohne_alles", "model.sdf"
    )
    default_robot_description = os.path.join(
        MATCH_MODELS_SHARE, "sdf", "match_drohne_alles", "match_drohne_alles.xacro"
    )

    declare_args = [
        DeclareLaunchArgument(
            "world",
            default_value="scale",
            description="Gazebo-Weltname / PX4_GZ_WORLD.",
        ),
        DeclareLaunchArgument("spawn_x", default_value="0.0", description="Spawn X."),
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
            "PX4_SYS_AUTOSTART": "40012",
            "PX4_SIM_MODEL": "match_drohne_alles",
            "PX4_SIMULATOR": "GZ",
            "PX4_GZ_MODEL_POSE": [
                spawn_x,
                TextSubstitution(text=","),
                spawn_y,
                TextSubstitution(text=","),
                spawn_z,
                TextSubstitution(text=",0,0,0"),
            ],
            "PX4_GZ_STANDALONE": "true",
            "PX4_GZ_WORLD": world,
            "PX4_HOME_LAT": "52.42449457140792",
            "PX4_HOME_LON": "9.620245153463955",
            "PX4_HOME_ALT": "20.0",
        },
    )

    mavros_timer = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "mavros",
                    "px4.launch",
                    ["fcu_url:=", "udp://:14540@"],
                ],
                output="screen",
            )
        ],
    )

    def make_spawn_timer(context, *args, **kwargs):
        spawn_process = ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "ros_gz_sim",
                "create",
                "-world",
                world,
                "-name",
                "match_drohne_alles",
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
        parameters=[{"robot_description": robot_description_content}],
    )

    bridge_arguments = [
        "/match_drohne_alles/front_rgb/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/match_drohne_alles/front_rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/match_drohne_alles/front_depth/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/match_drohne_alles/front_depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/match_drohne_alles/top_lidar/points@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    ]
    sensor_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_arguments,
        output="screen",
    )

    return LaunchDescription(
        declare_args
        + [px4_process, mavros_timer, spawn_action, robot_state_publisher_node, sensor_bridge_node]
    )
