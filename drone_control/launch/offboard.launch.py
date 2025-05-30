from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://:14540@127.0.0.1:14557',
            description='MAVLink connection URL'
        ),

        # MAVROS2 Node
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[
                {'fcu_url': LaunchConfiguration('fcu_url')},
                {'gcs_url': ''},
                {'target_system_id': 1},
                {'target_component_id': 1},
                {'system_id': 1},
                {'component_id': 1},
                {'use_sim_time': False},
                {'plugin_whitelist': [
                    'sys_status',
                    'sys_time',
                    'state',
                    'command',
                    'setpoint_position',
                    'setpoint_raw',
                    'setpoint_velocity',
                    'local_position',
                    'global_position',
                    'imu',
                    'rc_io'
                ]},
            ]
        ),

        # Dein eigenes Node
        Node(
            package='drone_control',
            executable='main',
            name='offboard_node',
            output='screen'
        )
    ])
