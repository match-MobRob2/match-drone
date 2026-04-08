import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


MATCH_MODELS_SHARE = get_package_share_directory("marvin_models")


def generate_launch_description():
    default_px4_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "PX4-Autopilot")
    )
    default_spawn_model = os.path.join(
        MATCH_MODELS_SHARE, "sdf", "marvin_drohne_lidar", "model.sdf"
    )

    declare_args = [
        DeclareLaunchArgument(
            "px4_dir",
            default_value=default_px4_dir,
            description="PX4-Arbeitsverzeichnis mit dem gebauten SITL-Binary.",
        ),
        DeclareLaunchArgument(
            "px4_autostart",
            default_value="40012",
            description="PX4 SYS_AUTOSTART Profil.",
        ),
        DeclareLaunchArgument(
            "px4_sim_model",
            default_value="marvin_drohne_lidar",
            description="PX4_SIM_MODEL Auswahl.",
        ),
        DeclareLaunchArgument(
            "px4_gz_world",
            default_value="scale",
            description="Name der GZ-Welt für PX4_GZ_WORLD.",
        ),
        DeclareLaunchArgument(
            "px4_gz_model_pose",
            default_value="2,2,0,0,0,0",
            description="PX4_GZ_MODEL_POSE (x,y,z,roll,pitch,yaw).",
        ),
        DeclareLaunchArgument(
            "px4_home_lat",
            default_value="52.42449457140792",
            description="PX4_HOME_LAT.",
        ),
        DeclareLaunchArgument(
            "px4_home_lon",
            default_value="9.620245153463955",
            description="PX4_HOME_LON.",
        ),
        DeclareLaunchArgument(
            "px4_home_alt",
            default_value="20.0",
            description="PX4_HOME_ALT.",
        ),
        DeclareLaunchArgument(
            "start_mavros",
            default_value="true",
            description="Starte MAVROS Bridge.",
        ),
        DeclareLaunchArgument(
            "mavros_fcu_url",
            default_value="udp://:14540@",
            description="MAVROS fcu_url Parameter.",
        ),
        DeclareLaunchArgument(
            "spawn_drone",
            default_value="true",
            description="Aktiviere das Spawnen des Modells.",
        ),
        DeclareLaunchArgument(
            "spawn_model",
            default_value=default_spawn_model,
            description="Pfad zur zu spawnenden SDF/URDF.",
        ),
        DeclareLaunchArgument(
            "spawn_name",
            default_value="marvin_drohne",
            description="Entity-Name in Gazebo.",
        ),
        DeclareLaunchArgument(
            "spawn_world",
            default_value="scale",
            description="Gazebo-Weltname für ros_gz_sim create.",
        ),
        DeclareLaunchArgument(
            "spawn_delay",
            default_value="3.0",
            description="Verzögerung in Sekunden vor dem Spawn-Aufruf.",
        ),
        DeclareLaunchArgument("spawn_x", default_value="0.0", description="Spawn X."),
        DeclareLaunchArgument("spawn_y", default_value="0.0", description="Spawn Y."),
        DeclareLaunchArgument("spawn_z", default_value="1.0", description="Spawn Z."),
        DeclareLaunchArgument("spawn_roll", default_value="0.0", description="Spawn Roll."),
        DeclareLaunchArgument("spawn_pitch", default_value="0.0", description="Spawn Pitch."),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0", description="Spawn Yaw."),
    ]

    px4_dir = LaunchConfiguration("px4_dir")
    px4_autostart = LaunchConfiguration("px4_autostart")
    px4_sim_model = LaunchConfiguration("px4_sim_model")
    px4_gz_world = LaunchConfiguration("px4_gz_world")
    px4_gz_model_pose = LaunchConfiguration("px4_gz_model_pose")
    px4_home_lat = LaunchConfiguration("px4_home_lat")
    px4_home_lon = LaunchConfiguration("px4_home_lon")
    px4_home_alt = LaunchConfiguration("px4_home_alt")

    start_mavros = LaunchConfiguration("start_mavros")
    mavros_fcu_url = LaunchConfiguration("mavros_fcu_url")

    spawn_world = LaunchConfiguration("spawn_world")
    spawn_name = LaunchConfiguration("spawn_name")
    spawn_model = LaunchConfiguration("spawn_model")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_roll = LaunchConfiguration("spawn_roll")
    spawn_pitch = LaunchConfiguration("spawn_pitch")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    px4_process = ExecuteProcess(
        cmd=["./build/px4_sitl_default/bin/px4"],
        cwd=px4_dir,
        output="screen",
        additional_env={
            "PX4_SYS_AUTOSTART": px4_autostart,
            "PX4_SIM_MODEL": px4_sim_model,
            "PX4_SIMULATOR": "GZ",
            "PX4_GZ_MODEL_POSE": px4_gz_model_pose,
            "PX4_GZ_STANDALONE": "true",
            "PX4_GZ_WORLD": px4_gz_world,
            "PX4_HOME_LAT": px4_home_lat,
            "PX4_HOME_LON": px4_home_lon,
            "PX4_HOME_ALT": px4_home_alt,
        },
    )

    mavros_timer = TimerAction(
        period=5.0,
        condition=IfCondition(start_mavros),
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "mavros",
                    "px4.launch",
                    ["fcu_url:=", mavros_fcu_url],
                ],
                output="screen",
            )
        ],
    )

    def make_spawn_timer(context, *args, **kwargs):
        spawn_enabled = LaunchConfiguration("spawn_drone").perform(context).lower()
        if spawn_enabled in ("false", "0"):
            return []

        delay_value = LaunchConfiguration("spawn_delay").perform(context)
        try:
            delay_sec = float(delay_value)
        except ValueError as exc:
            raise RuntimeError(
                f"spawn_delay muss eine Zahl sein, aktuell: {delay_value}"
            ) from exc

        spawn_process = ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "ros_gz_sim",
                "create",
                "-world",
                spawn_world,
                "-name",
                spawn_name,
                "-file",
                spawn_model,
                "-x",
                spawn_x,
                "-y",
                spawn_y,
                "-z",
                spawn_z,
                "-R",
                spawn_roll,
                "-P",
                spawn_pitch,
                "-Y",
                spawn_yaw,
            ],
            output="screen",
        )

        return [TimerAction(period=delay_sec, actions=[spawn_process])]

    spawn_action = OpaqueFunction(function=make_spawn_timer)

    return LaunchDescription(declare_args + [px4_process, mavros_timer, spawn_action])
