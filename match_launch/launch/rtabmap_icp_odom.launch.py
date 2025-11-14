from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ICP-Odometrie auf Punktwolke + IMU
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='rtab_icp_odometry',
            output='screen',
            parameters=[{
                # Frames
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',

                # Zeit
                'use_sim_time': True,

                # Sensor-Subscriptions
                'subscribe_scan': False,          # kein 2D-Laser
                'subscribe_scan_cloud': True,     # 3D-Cloud
                'subscribe_rgbd': False,
                'subscribe_rgb': False,
                'subscribe_stereo': False,
                'subscribe_imu': True,
                'subscribe_odometry': True,

                # IMU / Init
                'wait_imu_to_init': True,         # erst starten, wenn IMU da ist
                'qos_imu': 2,                     # SensorData QoS
                'queue_size': 100,

                # TF-Ausgabe
                'publish_tf': True,               # publisht odom -> base_link

                # Sync-Einstellungen
                'approx_sync': True,              # Pointcloud + IMU nicht exakt gleich getimed

                'Icp/PM': False,              # bool
                'Icp/Strategy': '0',            # int (0=PCL,1=PM,2=CCCoreLib)

                # ICP-Parameter (ALLE ALS STRING!)
                'Icp/MaxCorrespondenceDistance': '0.8',
                'Icp/CorrespondenceRatio': '0.03',
                'Icp/Iterations': '30',
                'Icp/VoxelSize': '0.1',
                'Icp/MaxTranslation': '2.0',
                'Icp/MaxRotation': '1.57',

                'Odom/GuessMotion': 'true',   # auch String
            }],
            remappings=[
                # 3D-Pointcloud deiner Depth-Cam
                ('scan_cloud', '/front_depth/points_filtered'),

                # IMU (mit sim-time-Zeitstempel)
                ('imu', '/mavros/imu/data_sim'),

                # Odom-Topic nach außen
                ('odom', '/rtabmap/odom'),

                # 2D-Laser-Eingang auf Dummy, damit er da nichts sucht
                ('scan', '/unused_scan'),
            ]
        ),
    ])
