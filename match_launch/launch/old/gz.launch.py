from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Fester Pfad zum PX4-Autopilot-Verzeichnis (z. B. relativ zum Launchfile)
    px4_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../src/PX4-Autopilot'))
    world = "default"       # oder z. B. "baylands"
    headless = "false"   # "true" für headless-Modus

    world_file = f"{px4_dir}/Tools/simulation/gz/worlds/{world}.sdf"
    model_file = "$(ros2 pkg prefix match_models)/share/match_models/urdf/iris.sdf"

    print(f"PX4 Directory: {px4_dir}")
    print(f"World File: {world_file}")
    print(f"Model File: {model_file}")


    return LaunchDescription([
        # PX4 SITL starten
        TimerAction(
            period=7.0,
            actions=[
                ExecuteProcess(
                cmd=["./build/px4_sitl_default/bin/px4"],
                cwd=px4_dir,
                output="screen",
                shell=True,
                additional_env={
                    "PX4_SYS_AUTOSTART": "40011",
                    "PX4_SIM_MODEL": "match_drohne",
                    "PX4_GZ_STANDALONE": "true",
                    "VERBOSE": "1"
                }
                )
            ]
        ),

        #Gazebo starten
        ExecuteProcess(
            cmd=[
                "bash", "-c",
                #f"source {px4_dir}/Tools/simulation/gazebo-classic/setup_gazebo.bash {px4_dir} {px4_dir}/build/px4_sitl_default && "
                f"export GZ_SIM_RESOURCE_PATH={px4_dir}/Tools/simulation/gz/models:$GZ_SIM_RESOURCE_PATH && "

                f"gz sim --verbose {px4_dir}/Tools/simulation/gz/worlds/{world}.world"
            ],
            output="screen"
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
        # TimerAction(
        #     period=15.0,
        #     actions=[
        #         Node(
        #             package='match_control',
        #             executable='main',
        #             name='match_control_node',
        #             output='screen',
        #         )
        #     ]
        # )


    ])
