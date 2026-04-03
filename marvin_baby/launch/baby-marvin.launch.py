import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare("marvin_baby")
    pkg_share_dir = get_package_share_directory("marvin_baby")

    match_models_share = get_package_share_directory("match_models")
    match_models_sdf_path = os.path.join(match_models_share, "sdf")

    # Workspace-Root: install/marvin_baby/share/marvin_baby → 4x dirname
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share_dir))))
    rgl_plugin_path = os.path.join(ws_root, "RGLGazeboPlugin", "build", "RGLServerPlugin")

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.dirname(pkg_share_dir) + ":" + match_models_sdf_path,
    )

    gz_server_config_path = SetEnvironmentVariable(
        name="GZ_SIM_SERVER_CONFIG_PATH",
        value=os.path.join(match_models_share, "config", "server.config"),
    )

    gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=rgl_plugin_path,
    )

    # ── Argumente ────────────────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="RViz starten",
    )
    use_rviz = LaunchConfiguration("use_rviz")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty.sdf",
        description="Gazebo-Weltdatei (default: empty.sdf = DART Physics wie im Demo)",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Gazebo-Simulationszeit verwenden",
    )

    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── URDF via xacro laden ─────────────────────────────────────────────────
    xacro_file = PathJoinSubstitution([pkg, "urdf", "marvin-baby.xacro"])
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    # ── Nodes ────────────────────────────────────────────────────────────────

    # 1) robot_state_publisher: publiziert /robot_description und TFs
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
    )

    # 2) Gazebo Harmonic starten (ros_gz_sim, wie im Rest des Projekts)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": ["-r ", world],
            "on_exit_shutdown": "True",
        }.items(),
    )

    # 3) Roboter in Gazebo spawnen (liest /robot_description)
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "mecanum_robot",
            "-topic", "/robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1",
        ],
        output="screen",
    )

    # 4) Bridges: Gazebo Harmonic → ROS2
    # Hinweis: /rgl_lidar (PointCloud2) wird vom RGL-Plugin direkt als ROS2-Topic
    # publiziert und braucht keinen Bridge.
    sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/rgl_lidar/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        output="screen",
    )

    # 5) joint_state_broadcaster aktivieren
    #    TimerAction: warten bis Gazebo + Controller Manager bereit sind
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            )
        ],
    )

    # 6) mecanum_drive_controller aktivieren
    mecanum_controller_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["mecanum_drive_controller", "--controller-manager", "/controller_manager"],
            )
        ],
    )

    # ── RViz (optional) ──────────────────────────────────────────────────────
    rviz_config = PathJoinSubstitution([pkg, "rviz", "robot.rviz"])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            gz_resource_path,
            gz_server_config_path,
            gz_plugin_path,
            use_rviz_arg,
            world_arg,
            use_sim_time_arg,
            robot_state_publisher,
            gazebo,
            spawn_robot,
            sensor_bridge,
            joint_state_broadcaster_spawner,
            mecanum_controller_spawner,
            rviz,
        ]
    )
