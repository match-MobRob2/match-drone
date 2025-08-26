from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    params = PathJoinSubstitution([FindPackageShare('match_slam'),
                                   'config', 'fast_lio2_params.yaml'])
    fastlio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        output='screen', 
        emulate_tty=True,
        parameters=[params, {'use_sim_time': True}],
        remappings=[
                # ('/Laser_map','/cloud_map'),
                ('/Odometry','/odom_lio')
                ],
        respawn=False, on_exit=Shutdown(),  # fail fast if it dies
    )
    return LaunchDescription([fastlio])
    
# def generate_launch_description():
#     # Your tuned params file (put the YAML in match_slam/config/)
#     config_params = PathJoinSubstitution([
#         FindPackageShare('match_slam'), 'config', 'fast_lio2_params.yaml'
#     ])

#     cfg_dir = PathJoinSubstitution([FindPackageShare('match_slam'), 'config'])
#     cfg_file = TextSubstitution(text='fast_lio2_params.yaml')

#     fastlio = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([FindPackageShare('fast_lio'), 'launch', 'mapping.launch.py'])
#         ),
#         launch_arguments={
#             'use_sim_time': 'true',
#             'config_path': cfg_dir,
#             'config_file': cfg_file,
#             'rviz': 'false',
#         }.items()
#     )

#     return LaunchDescription([
#         TimerAction(period=4.0, actions=[fastlio])  # wait for bridge/fixers
#     ])

    # return LaunchDescription([
    #     Node(
    #         package='fast_lio',
    #         executable='fastlio_mapping',   
    #         name='laser_mapping',       
    #         # namespace=LaunchConfiguration("namespace"),
    #         output="screen",
    #         emulate_tty=True,
    #         parameters=[config_params, {'use_sim_time': True, 'preprocess.lidar_type': 4}],
    #         # Optional: rename outputs if you prefer different topic names
    #         remappings=[
    #             ('/Laser_map', '/cloud_map'),
    #             ('/Odometry',  '/odom_lio'),
    #         ],
    #         respawn=True, respawn_delay=4.0,
    #         arguments=['--ros-args', '--log-level', 'laser_mapping:=error'],
    #     ),
    # ])