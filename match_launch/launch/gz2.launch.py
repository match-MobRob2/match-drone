from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Fester Pfad zum PX4-Autopilot-Verzeichnis (z. B. relativ zum Launchfile)
    px4_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../src/PX4-Autopilot'))
    world = "default"       # oder z. B. "baylands"
    headless = "false"   # "true" für headless-Modus

    match_models_path = get_package_share_directory("match_models")


    world_file = f"{px4_dir}/Tools/simulation/gz/worlds/{world}.sdf"
    model_file = "$(ros2 pkg prefix match_models)/share/match_models/urdf/iris.sdf"


    print(f"PX4 Directory: {px4_dir}")
    print(f"World File: {world_file}")
    print(f"Model File: {model_file}")
    
    PX4_HOME_LAT = "52.42449457140792"  # Beispielkoordinaten
    PX4_HOME_LON = "9.620245153463955"
    PX4_HOME_ALT = "20.0"  # Beispielhöhe in Metern

    
    px4_gz_models = f"{px4_dir}/Tools/simulation/gz/models"
    px4_gz_worlds = f"{px4_dir}/Tools/simulation/gz/worlds"
    px4_gz_plugins = f"{px4_dir}/build/px4_sitl_default/src/modules/simulation/gz_plugins"
    px4_gz_server_config = f"{px4_dir}/src/modules/simulation/gz_bridge/server.config"

    # Umgebungsvariablen erweitern
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

    gz_sim_resource_path = f"{px4_gz_worlds}:{px4_gz_plugins}"

    world = f"{match_models_path}/worlds/empty.sdf"

    print(f"World: {world}")



    #52.63098207727893, 10.05284961998495
    # PX4_HOME_LAT = "52.63098207727893"  # Beispielkoordinaten
    # PX4_HOME_LON = "10.05284961998495"

    return LaunchDescription([
        #PX4 SITL starten
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    name="px4_sitl",
                    cmd=["./build/px4_sitl_default/bin/px4"],
                    cwd=px4_dir,
                    output="screen",
                    shell=True,
                    additional_env={
                        "PX4_SYS_AUTOSTART": "40011",
                        "PX4_SIM_MODEL": "match_drohne",
                        "PX4_SIMULATOR": "GZ",
                        "VERBOSE": "1",
                        "PX4_GZ_STANDALONE": "true",
                        "PX4_HOME_LAT": PX4_HOME_LAT,
                        "PX4_HOME_LON": PX4_HOME_LON,
                        "PX4_HOME_ALT": PX4_HOME_ALT,
                        "PX4_GZ_WORLD": "empty",
                    }
                )
            ]
        ),

        #Gazebo starten
        ExecuteProcess(
            name="gazebo_server",
            cmd=[
                "bash", "-c",
                f"echo $PX4_GZ_MODELS && "
                f"echo $PX4_GZ_WORLDS && "
                f"echo $PX4_GZ_PLUGINS && "
                f"echo $PX4_GZ_SERVER_CONFIG && "
                f"echo $GZ_SIM_RESOURCE_PATH && "
                f"echo $GZ_SIM_SYSTEM_PLUGIN_PATH && "
                f"echo $GZ_SIM_SERVER_CONFIG_PATH && "
                f"gz sim --verbose=1 -r -s {world}"
            ],
            additional_env = {
                "PX4_GZ_MODELS": px4_gz_models,
                "PX4_GZ_WORLDS": px4_gz_worlds,
                "PX4_GZ_PLUGINS": px4_gz_plugins,
                "PX4_GZ_SERVER_CONFIG": px4_gz_server_config,
                "GZ_SIM_RESOURCE_PATH": gz_resource_path,
                "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
                "GZ_SIM_SERVER_CONFIG_PATH": gz_server_config_path,
            },
            output="screen"
        ),

        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    name="gazebo_client",
                    cmd=[
                        "gz", "sim", "-g"
                    ],
                    output="screen"
                ),
            ]
        ),

        # # Modell nach 4 Sekunden spawnen
        # TimerAction(
        #     period=1.0,
        #     actions=[
        #     ExecuteProcess(
        #         cmd=[
        #         "bash", "-c",
        #         f"gz service -s /world/default/create "
        #         f"--reqtype gz.msgs.EntityFactory "
        #         f"--reptype gz.msgs.Boolean "
        #         f"--timeout 5000 "
        #         f"--req 'sdf_filename: \"/home/luca/Match_Drohne/src/match_models/urdf/x500_base/model.sdf\", "
        #         f"name: \"match_drohne\", "
        #         f"pose: {{position: {{x: 1.01, y: 0.98, z: 0.83}}}}'"
        #         ],
        #         output="screen"
        #     )
        #     ]
        # ),

        # #Mavros starten
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "launch", "mavros", "px4.launch",
                        "fcu_url:=udp://:14540@"
                    ],
                    output="screen"
                )
            ]
        ),

        #GUI starten
        # TimerAction(
        #     period=6.0,
        #     actions=[
        #         ExecuteProcess(
        #             cmd=[
        #                 "bash", "-c",
        #                 f"source {px4_dir}/Tools/simulation/gazebo-classic/setup_gazebo.bash {px4_dir} {px4_dir}/build/px4_sitl_default && "
        #                 f"gzclient --verbose"],
        #             output="screen"
        #         )
        #     ]
        # ),

        # Match Control Node starten
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='match_control',
                    executable='abheben_vor_landen',
                    name='match_control_node',
                    output='screen',
                )
            ]
        )


    ])
