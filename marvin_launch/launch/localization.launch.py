"""Flug auf gespeicherter Karte: Relokalisierung gegen map.pcd, Planer mit map.bt.

    ros2 launch marvin_launch localization.launch.py map_dir:=~/karten/halle \\
        initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0 [sim:=false ...]

initial_* = Startpose der Drohne im Karten-Frame (Karten-Ursprung = Startpunkt
des Mapping-Flugs). Die Relokalisierung verfeinert sie per GICP; grob (~1 m,
~15°) reicht. Alle anderen Argumente gehen an nav_fastlio.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    d = os.path.expanduser(LaunchConfiguration('map_dir').perform(context))
    for f in ('map.pcd', 'map.bt'):
        if not os.path.isfile(os.path.join(d, f)):
            raise RuntimeError(f'{d}/{f} fehlt — erst mapping.launch.py + map_optimizer')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('marvin_launch'), 'launch', 'nav_fastlio.launch.py')),
        launch_arguments={
            'map_pcd': os.path.join(d, 'map.pcd'),
            'map_bt': os.path.join(d, 'map.bt'),
        }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_dir', description='Ordner mit map.pcd + map.bt (vom map_optimizer)'),
        OpaqueFunction(function=setup),
    ])
