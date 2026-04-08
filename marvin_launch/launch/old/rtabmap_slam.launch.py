from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{                # Frames
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'publish_tf': True,

                # Zeit
                'use_sim_time': True,

                # Subscriptions
                'subscribe_scan': False,
                'subscribe_scan_cloud': True,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_rgbd': False,
                'subscribe_stereo': False,
                'subscribe_odom_info': False,
                'subscribe_odom': False,

                # Sync
                'approx_sync': True,

                # RTAB-Map interne Parameter (als STRING!)
                'Grid/3D': 'true',               # vorher True
                'Grid/RangeMax': '15',           # als String
                'Rtabmap/DetectionRate': '5.0',  # als String
                'Reg/Strategy': '1',             # ICP
                'Vis/FeatureType': '0',          # keine visuellen Features

                'wait_for_transform': 2.0,     # Sekunden
            }],
            remappings=[
                ('scan_cloud', '/front_depth/points_filtered'),
                ('imu', '/mavros/imu/data_sim'),
                ('odom', '/rtabmap/odom'),
            ]
        )
    ])
