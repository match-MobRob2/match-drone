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
                'labyrinth.launch.py'
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
                        'match_drohne_alles_lab.launch.py'
                    )
                )
            )
        ]
    )

    mavros_local_to_tf = Node(
        package='match_utils',
        executable='mavros_local_to_tf',
        name='mavros_local_to_tf',
        output='screen',
        parameters=[{"use_sim_time": True}]
    )

    map_lidar_to_camera = Node(
        package='match_utils',
        executable='map_livox_to_realsense',
        name='map_livox_to_realsense',
        output='screen',
        parameters=[{"use_sim_time": True}]
    )

    relay_lidar = Node(
        package='topic_tools',
        executable='relay',
        name='relay_lidar',
        output='screen',
        parameters=[{"use_sim_time": True}],
        remappings=[
            ('input', '/rgl_lidar'),
            ('output', '/cloud_merged'),
        ]
    )

    relay_realsense = Node(
        package='topic_tools',
        executable='relay',
        name='relay_realsense',
        output='screen',
        parameters=[{"use_sim_time": True}],
        remappings=[
            ('input', '/match_drohne_alles/front_depth/image/points'),
            ('output', '/cloud_merged'),
        ]
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
                parameters=[
                    {
                        "use_sim_time": True,
                        "resolution": 0.3,
                        "frame_id": "map",

                        "filter_ground": True,

                        # --- Sensor Model ---
                        # Conservative hit → needs 2-3 consistent observations
                        # to mark occupied (reduces phantom voxels from timing jitter)
                        "sensor_model.hit": 0.6,
                        # Moderate miss → doesn't flip occupied voxels too eagerly
                        "sensor_model.miss": 0.35,
                        # Clamp range: prevents runaway confidence in either direction
                        "sensor_model.min": 0.12,
                        "sensor_model.max": 0.90,
                        # Reduced range → less noise from distant points
                        "sensor_model.max_range": 15.0,

                        # --- Z Filters ---
                        # Insertion filter: drop ground returns and points above drone ceiling
                        # Drone flies at 0-12m, body is ~0.2m thick → start at 0.2m above ground
                        "pointcloud_min_z": -2.0,
                        "pointcloud_max_z": 15.0,
                        # Occupancy publishing: same range, filtered for planner consumption
                        "occupancy_min_z": -2.0,
                        "occupancy_max_z": 15.0,

                        # --- Performance ---
                        # false = publish on every change, not just on subscriber connect
                        # critical for a drone that moves constantly
                        "latch": True,
                    }
                ],
                remappings=[
                    ('cloud_in', '/cloud_merged'),
                ]
            )
        ]
    )

    return LaunchDescription([
        world_launch,
        drone_launch,
        mavros_local_to_tf,
        map_lidar_to_camera,
        relay_lidar,
        relay_realsense,
        octomap_server,
    ])
