from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable, TextSubstitution
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    # --- Paths / config ---
    package_name = 'match_slam'
    pkg_share = FindPackageShare(package_name)
    px4_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../src/PX4-Autopilot'))

    # World selection
    match_models_path = get_package_share_directory("match_models")
    empty_world = f"{match_models_path}/worlds/empty.sdf"
    beste_welt_world = f"{match_models_path}/worlds/beste_welt.sdf"
    px4_world = f"{px4_dir}/Tools/simulation/gz/worlds/walls.sdf"

    PX4_HOME_LAT = "52.42449457140792"
    PX4_HOME_LON = "9.620245153463955"
    PX4_HOME_ALT = "20.0"

    px4_gz_models = f"{px4_dir}/Tools/simulation/gz/models"
    px4_gz_worlds = f"{px4_dir}/Tools/simulation/gz/worlds"
    px4_gz_plugins = f"{px4_dir}/build/px4_sitl_default/src/modules/simulation/gz_plugins"
    px4_gz_server_config = f"{px4_dir}/src/modules/simulation/gz_bridge/server.config"

    # Extend environment for Gazebo (Harmonic / gz-sim)
    prev = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = f"{(prev + ':') if prev else ''}{px4_gz_worlds}:{px4_gz_models}"
    prev = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    gz_plugin_path = f"{(prev + ':') if prev else ''}{px4_gz_plugins}"
    prev = os.environ.get('GZ_SIM_SERVER_CONFIG_PATH', '')
    gz_server_config_path = f"{(prev + ':') if prev else ''}{px4_gz_server_config}"

    print(f"PX4 Directory: {px4_dir}")
    # print(f"World: {empty_world}")
    print(f"PX4 GZ Models: {px4_gz_models}")
    print(f"PX4 GZ Worlds: {px4_gz_worlds}")
    print(f"PX4 GZ Plugins: {px4_gz_plugins}")
    print(f"PX4 GZ Server Config: {px4_gz_server_config}")
    print(f"GZ Resource Path: {gz_resource_path}")
    print(f"GZ Plugin Path: {gz_plugin_path}")
    print(f"GZ Server Config Path: {gz_server_config_path}")

    # 1) Gazebo server - start immediately
    gazebo_server = ExecuteProcess(
        name="gazebo_server",
        cmd=[
            "bash", "-c",
            f"echo $PX4_GZ_MODELS && "
            f"echo $PX4_GZ_WORLDS && "
            f"echo $PX4_GZ_PLUGINS && "
            f"echo $PX4_GZ_SERVER_CONFIG && "
            f"echo $GZ_SIM_RESOURCE_PATH && "
            f"echo $GZ_SIM_SYSTEM_PLUGIN_PATH && "
            f"echo $GZ_SIM_SERVER_CONFIG_PATH && "
            f"gz sim --verbose=1 -r -s {px4_world}"
        ],
        additional_env={
            "PX4_GZ_MODELS": px4_gz_models,
            "PX4_GZ_WORLDS": px4_gz_worlds,
            "PX4_GZ_PLUGINS": px4_gz_plugins,
            "PX4_GZ_SERVER_CONFIG": px4_gz_server_config,
            "GZ_SIM_RESOURCE_PATH": gz_resource_path,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
            "GZ_SIM_SERVER_CONFIG_PATH": gz_server_config_path,
        },
        output="screen"
    )

    # 2) ROS bridge
    ros_bridge = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name="gz_bridge_points",
                arguments=[
                    '/gz_scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                    '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                    '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                    '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                    '/livox/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                    # '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    # '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
                ],
                remappings=[('/gz_scan', '/scan')],
                parameters=[{'use_sim_time': True}],
                output="screen",
            )
        ]
    )

    # 3) PointCloud+IMU topic name fixes
    pcl_topic_fix_node = TimerAction(
        period=3.5,
        actions=[
            Node(
                package='sim_utils',
                executable='pc_frame_fix',
                name='pc_frame_fix',
                parameters=[{'use_sim_time': True, 'out_frame': 'lidar_link'}],
                output='screen'
            )
        ]
    )

    imu_topic_fix_node = TimerAction(
        period=3.5,
        actions=[
            Node(
                package='sim_utils',
                executable='imu_frame_fix',
                name='imu_frame_fix',
                parameters=[
                    {'use_sim_time': True},
                    {'target_frame': 'lidar_link'},
                    {'in_topic': '/livox/imu'},
                    {'out_topic': '/livox/imu_fixed'},
                ],
                output='screen'
            )
        ]
    )

    # 4) robot_state_publisher - shortly after server so TFs are available early
    rsp_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'robot_state_publisher.launch.py'])
        ),
        launch_arguments={
            'sim_mode': 'true',
            'robot_name': 'match_drohne1',
            'use_sim_time': 'true',
        }.items()
    )

    # 5) Gazebo client (GUI)
    gazebo_client = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                name="gazebo_client",
                cmd=["gz", "sim", "-g"],  # GUI on (use "-f" for headless)
                output="screen"
            ),
        ]
    )

    # 6) PX4 SITL - after Gazebo is up
    px4_sitl = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                name="px4_sitl",
                cmd=["./build/px4_sitl_default/bin/px4"],
                cwd=px4_dir,
                output="screen",
                shell=True,
                additional_env={
                    "PX4_SYS_AUTOSTART": "4001",
                    "PX4_SIM_MODEL": "match_drohne",
                    "PX4_SIMULATOR": "GZ",
                    "VERBOSE": "1",
                    "PX4_GZ_STANDALONE": "false",
                    "PX4_HOME_LAT": PX4_HOME_LAT,
                    "PX4_HOME_LON": PX4_HOME_LON,
                    "PX4_HOME_ALT": PX4_HOME_ALT,
                    "PX4_GZ_WORLD": "walls",   
                    "PX4_GZ_WORLDS": match_models_path,  # so Gazebo can find worlds/garbsen.sdf
                    "GZ_SIM_RESOURCE_PATH": gz_resource_path, 
                }
            )
        ]
    )

    # 7) MAVROS - after PX4 is running
    mavros_node = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "launch", "mavros", "px4.launch", 
                    "fcu_url:=udp://:14540@"
                ],
                output="screen"
            )
        ]
    )

    # 8) Match control node - after everything is ready
    control_node = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='match_control',
                executable='main',
                name='match_control_node',
                output='screen',
                # parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # 9) --- RViz2 ---
    rviz2 = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        gazebo_server,
        ros_bridge,
        pcl_topic_fix_node,
        imu_topic_fix_node,
        rsp_node,
        gazebo_client,
        px4_sitl,
        mavros_node,
        control_node,
        rviz2,
    ])
