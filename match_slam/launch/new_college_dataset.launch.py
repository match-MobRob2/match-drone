from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable, TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    package_name = 'match_slam'
    pkg_share = FindPackageShare(package_name)

    return LaunchDescription([

        # Arguments
        DeclareLaunchArgument('pcd_dir', default_value='/path/to/sequence/pcds'),
        DeclareLaunchArgument('imu_csv', default_value='/path/to/imu.csv'),
        DeclareLaunchArgument('gt_map', default_value='/path/to/groundtruth_map.pcd'),

        # Replay dataset (LiDAR + IMU)
        Node(
            package='sim_utils',
            executable='pcd_replay',
            name='pcd_replay',
            parameters=[
                {'pcd_dir': LaunchConfiguration('pcd_dir')},
                {'imu_csv': LaunchConfiguration('imu_csv')},
                {'frame_id': 'os_lidar'},
                {'rate_hz': 10.0}
            ],
            output='screen'
        ),

        # Publish ground-truth map
        Node(
            package='sim_utils',
            executable='gt_map_pub',
            name='gt_map_pub',
            parameters=[
                {'map_file': LaunchConfiguration('gt_map')},
                {'frame_id': 'map'}
            ],
            output='screen'
        ),

        # FAST-LIO2 node (assumes installed as ros2 package)
        Node(
            package='fast_lio2',
            executable='fastlio_mapping',
            name='fastlio2',
            output='screen',
            parameters=['fast_lio2_params.yaml'],
        ),

        # TF broadcaster (odometry → base_link)
        Node(
            package='sim_utils',
            executable='odom_tf',
            name='odom_tf',
            output='screen'
        ),

        # Robot model publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'robot_state_publisher.launch.py'])
            ),
            launch_arguments={
                'sim_mode': 'true',
                'robot_name': 'match_drohne1',
                'use_sim_time': 'true',
            }.items()
        ),

        # TF broadcaster from FAST-LIO2 odometry
        Node(
            package='sim_utils',
            executable='odom_tf',
            name='odom_tf',
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'new_college_dataset.rviz'])],
        ),
    ])
