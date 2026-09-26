"""Dauerdienst auf Sim- bzw. Drohnen-PC: rosbridge + Supervisor (inkl. Webserver).

    ros2 launch marvin_ui bringup.launch.py              # Sim
    ros2 launch marvin_ui bringup.launch.py sim:=false   # echte Drohne

Dann im Browser (auch von anderen Rechnern im Netz): http://<rechner>:8088
Alles Weitere (Stack starten, Karten, Relokalisieren, Missionen) laeuft ueber die Seite.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = LaunchConfiguration('sim')
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('maps_dir', default_value='', description='leer = <ws>/maps'),
        DeclareLaunchArgument('http_port', default_value='8088'),
        DeclareLaunchArgument('launch_extra', default_value='',
                              description='Zusatz-Argumente fuer jeden Stack-Start, z.B. "lookahead:=1.5"'),
        Node(package='rosbridge_server', executable='rosbridge_websocket', name='rosbridge',
             parameters=[{'port': 9090, 'address': ''}], output='log'),
        DeclareLaunchArgument('camera_topic', default_value='', description='leer = Sim-/RealSense-RGB je nach sim'),
        # Kamera-Stream als MJPEG; kostet nur CPU, solange die Seite das Bild anzeigt
        Node(package='web_video_server', executable='web_video_server', name='web_video_server',
             parameters=[{'port': 8089, 'address': '0.0.0.0'}], output='log'),
        Node(package='marvin_ui', executable='supervisor', name='marvin_supervisor', output='screen',
             parameters=[{
                 'sim': ParameterValue(sim, value_type=bool),
                 'http_port': ParameterValue(LaunchConfiguration('http_port'), value_type=int),
                 'launch_extra': LaunchConfiguration('launch_extra'),
                 'maps_dir': LaunchConfiguration('maps_dir'),
                 'camera_topic': LaunchConfiguration('camera_topic'),
             }]),
    ])
