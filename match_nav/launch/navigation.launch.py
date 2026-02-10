#!/usr/bin/env python3
"""
Navigation Launch File for Match Drone

Starts complete navigation stack:
- Includes mapper.launch.py (SLAM stack)
- Octomap Server
- Global Planner
- Waypoint Follower
- Mission Manager

Author: Match Drone Team
License: MIT
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    enable_octomap_arg = DeclareLaunchArgument(
        'enable_octomap',
        default_value='true',
        description='Enable Octomap Server'
    )

    # Get package directories
    match_nav_dir = get_package_share_directory('match_nav')
    match_launch_dir = get_package_share_directory('match_launch')

    # Configuration files
    octomap_config = PathJoinSubstitution([
        FindPackageShare('match_nav'),
        'config', 'octomap.yaml'
    ])

    planner_config = PathJoinSubstitution([
        FindPackageShare('match_nav'),
        'config', 'planner.yaml'
    ])

    navigation_config = PathJoinSubstitution([
        FindPackageShare('match_nav'),
        'config', 'navigation.yaml'
    ])

    # Include mapper.launch.py (starts SLAM stack: Gazebo, Drone, FAST-LIO, RTAB-Map)
    mapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(match_launch_dir, 'launch', 'mapper.launch.py')
        )
    )

    # Octomap Server (Phase 1: Commented out until octomap_server is installed)
    # TODO: Uncomment after installing: sudo apt install ros-humble-octomap-server
    # octomap_server = Node(
    #     package='octomap_server',
    #     executable='octomap_server_node',
    #     name='octomap_server',
    #     parameters=[octomap_config,
    #                 {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    #     remappings=[
    #         ('cloud_in', '/cloud_registered')
    #     ],
    #     output='screen'
    # )

    # Global Planner Node
    global_planner = Node(
        package='match_nav',
        executable='global_planner',
        name='global_planner',
        parameters=[planner_config,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Waypoint Follower Node
    waypoint_follower = Node(
        package='match_nav',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[navigation_config,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Mission Manager Node
    mission_manager = Node(
        package='match_nav',
        executable='mission_manager',
        name='mission_manager',
        parameters=[navigation_config,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        enable_octomap_arg,

        # Include SLAM stack
        mapper_launch,

        # Navigation nodes (Phase 1 MVP)
        # octomap_server,  # TODO: Uncomment when octomap_server installed
        global_planner,
        waypoint_follower,
        mission_manager,
    ])
