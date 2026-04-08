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
                get_package_share_directory('marvin_launch'),
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
                        get_package_share_directory('marvin_launch'),
                        'launch',
                        'marvin_drohne_alles.launch.py'
                    )
                )
            )
        ]
    )

    mavros_local_to_tf = Node(
        package='marvin_utils',
        executable='mavros_local_to_tf',
        name='mavros_local_to_tf',
        output='screen',
        parameters=[{"use_sim_time": True}]
    )

    map_lidar_to_camera = Node(
        package='marvin_utils',
        executable='map_livox_to_realsense',
        name='map_livox_to_realsense',
        output='screen',
        parameters=[{"use_sim_time": True}]
    )

    #ros2 run octomap_server octomap_server_node --ros-args   -p resolution:=0.2   -p frame_id:=map   -p sensor_model.max_range:=35.0   -p occupancy_min_z:=-1.0   -p occupancy_max_z:=15.0   -r cloud_in:=/rgl_lidar
    octomap_server = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='octomap_server',
                executable='octomap_server_node',
                name='octomap_server',
                output='screen',
                parameters=[{"use_sim_time": True}],
                arguments=[
                    '--ros-args',
                    '-p', 'resolution:=0.2',
                    '-p', 'frame_id:=map',
                    '-p', 'sensor_model.max_range:=35.0',
                    '-p', 'occupancy_min_z:=-1.0',
                    '-p', 'occupancy_max_z:=15.0',
                    '-r', 'cloud_in:=/rgl_lidar',
                ]
            )
        ]
    )
    # mapper_planner_node = Node(
    #     package='uav_mapper_planner',
    #     executable='mapper_planner_node',
    #     name='mapper_planner',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}],
    # )
 

    return LaunchDescription([
        world_launch,
        drone_launch,
        mavros_local_to_tf,
        map_lidar_to_camera,
        octomap_server,
    ])
