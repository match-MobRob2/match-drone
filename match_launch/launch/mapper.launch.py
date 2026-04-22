from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('match_launch'),
                'launch',
                'world2.launch.py'
            )
        )
    )

    drone_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('match_launch'),
                        'launch',
                        'match_drohne_alles.launch.py'
                    )
                )
            )
        ]
    )

    fastlio = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('fast_lio'),
                        'launch',
                        'mapping.launch.py'
                    )
                ),
                launch_arguments={
                    'config_file': '/home/luca/match_drone/src/FAST_LIO_ROS2/config/mid360.yaml'
                }.items()
            )
        ]
    )

    odom_to_drone = Node(
        package='match_utils',
        executable='odometry_to_drone',
        name='odometry_to_drone',
        output='screen',
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('match_launch'),
                'launch',
                'rtabmap_rgbd_fastlio.launch.py'
            )
        )
    )



    return LaunchDescription([
        world_launch,
        drone_launch,
        fastlio,
        odom_to_drone,
        rtabmap
    ])
