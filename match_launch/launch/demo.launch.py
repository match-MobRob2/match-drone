from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Fester Pfad zum PX4-Autopilot-Verzeichnis (z. B. relativ zum Launchfile)
    px4_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../src/PX4-Autopilot'))
    world = "empty"       # oder z. B. "baylands"
    headless = "false"   # "true" für headless-Modus

    print(f"PX4 Directory: {px4_dir}")

    return LaunchDescription([
        # PX4 SITL starten
        ExecuteProcess(
            cmd=[
                "bash", "-c",
                f"cd {px4_dir} && "
                "PX4_SYS_AUTOSTART=10015 "
                "PX4_SIM_MODEL=match_drohne "
                "PX4_SIMULATOR=gazebo-classic "
                "./build/px4_sitl_default/bin/px4"
            ],
            output="screen"
        ),

        # Gazebo starten
        ExecuteProcess(
            cmd=[
                "bash", "-c",
                f"source {px4_dir}/Tools/simulation/gazebo-classic/setup_gazebo.bash {px4_dir} {px4_dir}/build/px4_sitl_default && "
                f"gzserver --verbose {px4_dir}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/{world}.world"
            ],
            output="screen"
        ),

        # Modell nach 4 Sekunden spawnen
        TimerAction(
            period=4.0,
            actions=[
            ExecuteProcess(
                cmd=[
                "bash", "-c",
                f"source {px4_dir}/Tools/simulation/gazebo-classic/setup_gazebo.bash {px4_dir} {px4_dir}/build/px4_sitl_default && "
                f"gz model --verbose "
                f"--spawn-file $(ros2 pkg prefix match_models)/share/match_models/urdf/iris.sdf "
                "--model-name match_drohne -x 1.01 -y 0.98 -z 0.83"
                ],
                output="screen"
            )
            ]
        ),

        # Mavros starten
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "launch", "mavros", "px4.launch",
                        "fcu_url:=udp://:14540@127.0.0.1:14557"
                    ],
                    output="screen"
                )
            ]
        ),

        #GUI starten
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "bash", "-c",
                        f"source {px4_dir}/Tools/simulation/gazebo-classic/setup_gazebo.bash {px4_dir} {px4_dir}/build/px4_sitl_default && "
                        f"gzclient --verbose"],
                    output="screen"
                )
            ]
        ),

        # Match Control Node starten
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='match_control',
                    executable='main',
                    name='match_control_node',
                    output='screen',
                )
            ]
        )


    ])
