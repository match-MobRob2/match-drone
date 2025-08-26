#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('match_slam')
    xacro_file = os.path.join(pkg_share, 'description', 'urdf', 'drone.urdf.xacro')

    # --- Launch arguments ---
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',
        description='Enable simulation mode (PX4 SITL)'
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='match_drohne',
        description='Robot namespace'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # --- Generate robot_description from Xacro ---
    robot_description_content = Command([
        'xacro', ' ', xacro_file,
        ' sim_mode:=', LaunchConfiguration('sim_mode'),
        ' robot_name:=', LaunchConfiguration('robot_name'),
    ])

    robot_description = ParameterValue(robot_description_content, value_type=str)

    # --- Robot State Publisher ---
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # --- Joint State Publisher GUI ---
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        sim_mode_arg,
        robot_name_arg,
        use_sim_time_arg,
        node_robot_state_publisher,
        # node_joint_state_publisher_gui
    ])