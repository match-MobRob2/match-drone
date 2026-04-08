import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


MATCH_MODELS_SHARE = get_package_share_directory("marvin_models")


def generate_launch_description():
    default_px4_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "PX4-Autopilot")
    )
    default_world = os.path.join(MATCH_MODELS_SHARE, "worlds", "scale.sdf")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Vollständiger Pfad zur zu ladenden GZ-Welt (SDF).",
    )
    declare_gui = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Starte den GZ GUI-Client.",
    )
    declare_verbosity = DeclareLaunchArgument(
        "verbosity",
        default_value="3",
        description="GZ Sim Verbosity (0-4).",
    )

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    verbosity = LaunchConfiguration("verbosity")

    px4_gz_models = f"{default_px4_dir}/Tools/simulation/gz/models"
    px4_gz_worlds = f"{default_px4_dir}/Tools/simulation/gz/worlds"
    px4_gz_plugins = f"{default_px4_dir}/build/px4_sitl_default/src/modules/simulation/gz_plugins"
    px4_gz_server_config = f"{default_px4_dir}/src/modules/simulation/gz_bridge/server.config"


    def build_path(existing_value, *paths):
        parts = [existing_value] if existing_value else []
        parts.extend(paths)
        return ":".join(filter(None, parts))

    gz_resource_path = f"{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}{px4_gz_worlds}:{px4_gz_models}"
    gz_plugin_path = f"{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}{px4_gz_plugins}"
    gz_server_config_path = f"{os.environ.get('GZ_SIM_SERVER_CONFIG_PATH', '')}{px4_gz_server_config}"

    print(f"PX4 GZ Models: {px4_gz_models}")
    print(f"PX4 GZ Worlds: {px4_gz_worlds}")
    print(f"PX4 GZ Plugins: {px4_gz_plugins}")
    print(f"PX4 GZ Server Config: {px4_gz_server_config}")

    print(f"GZ Resource Path: {gz_resource_path}")
    print(f"GZ Plugin Path: {gz_plugin_path}")
    print(f"GZ Server Config Path: {gz_server_config_path}")

    gz_server = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world, "-v", verbosity],
        additional_env={
            "PX4_GZ_MODELS": px4_gz_models,
            "PX4_GZ_WORLDS": px4_gz_worlds,
            "PX4_GZ_PLUGINS": px4_gz_plugins,
            "PX4_GZ_SERVER_CONFIG": px4_gz_server_config,
            "GZ_SIM_RESOURCE_PATH": gz_resource_path,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
            "GZ_SIM_SERVER_CONFIG_PATH": gz_server_config_path,
        },
        output="screen",
    )

    gz_client = TimerAction(
        period=2.0,
        condition=IfCondition(gui),
        actions=[
            ExecuteProcess(
                cmd=["gz", "sim", "-g", "-v", verbosity],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            declare_world,
            declare_gui,
            declare_verbosity,
            gz_server,
            gz_client,
        ]
    )
