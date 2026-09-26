"""Mapping-Flug: nav_fastlio im Echtzeit-Modus + Keyframe-Aufnahme.

    ros2 launch marvin_launch mapping.launch.py map_dir:=~/karten/halle [sim:=false ...]
    # fliegen (Start-Umgebung am Ende nochmal anfliegen -> Loop-Closure), Ctrl-C
    ros2 run marvin_nav map_optimizer ~/karten/halle     # -> map.pcd, map.bt
    ros2 launch marvin_launch localization.launch.py map_dir:=~/karten/halle

map_dir muss neu/leer sein. Zusaetzlich (optional, zum Vergleich) lassen sich
die unoptimierten Online-Karten speichern:
    ros2 service call /lidar_relocalization/save_map std_srvs/srv/Trigger  # -> online.pcd
    ros2 service call /nav_node/save_map std_srvs/srv/Trigger              # -> online.bt
Alle anderen Argumente (sim, gps, world, spawn_*, ...) gehen an nav_fastlio.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    d = os.path.expanduser(LaunchConfiguration('map_dir').perform(context))
    if os.path.isdir(d) and os.listdir(d):
        raise RuntimeError(f'map_dir {d} ist nicht leer — neuen Ordner waehlen (nichts wird ueberschrieben)')
    os.makedirs(d, exist_ok=True)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('marvin_launch'), 'launch', 'nav_fastlio.launch.py')),
        launch_arguments={
            'map_pcd': '',
            'keyframe_dir': os.path.join(d, 'keyframes'),
            'map_pcd_save': os.path.join(d, 'online.pcd'),
            'map_bt': os.path.join(d, 'online.bt'),
        }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_dir', description='Neuer Ordner fuer Keyframes + Karten'),
        OpaqueFunction(function=setup),
    ])
