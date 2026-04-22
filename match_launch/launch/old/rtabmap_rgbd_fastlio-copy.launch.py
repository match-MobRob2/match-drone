# rtabmap_rgbd_fastlio_optimized.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = True

    # 1) RGBD Synchronisation - OPTIMIERT
    rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            
            # WICHTIG: Approximative Sync mit größerem Zeitfenster
            "approx_sync": True,
            "approx_sync_max_interval": 0.1,  # 100ms Toleranz
            
            # Queue-Größe reduzieren für weniger Latenz
            "queue_size": 10,
            
            # Compressed Transport für schnellere Übertragung
            "compressed": False,  # nur bei Netzwerk-Problemen auf True
        }],
        remappings=[
            ("rgb/image",       "/match_drohne_alles/front_rgb/image"),
            ("rgb/camera_info", "/match_drohne_alles/front_rgb/camera_info"),
            ("depth/image",     "/match_drohne_alles/front_depth/image"),
            ("depth/camera_info", "/match_drohne_alles/front_depth/camera_info"),
            ("rgbd_image",      "/rtabmap/rgbd_image"),
        ]
    )

    # 2) RTAB-Map SLAM - PERFORMANCE-OPTIMIERT
    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,

            # ============ FRAMES ============
            "frame_id": "body",
            "odom_frame_id": "camera_init",
            "map_frame_id": "map",

            # ============ SUBSCRIPTIONS ============
            "subscribe_rgbd": True,
            "subscribe_depth": False,
            "subscribe_rgb": False,
            "subscribe_scan": False,
            
            # ============ PERFORMANCE-KRITISCH ============
            # Detektionsrate reduzieren für bessere Performance
            "Rtabmap/DetectionRate": "5.0",  # runter von 10 Hz
            
            # Queue deutlich kleiner für weniger Latenz
            "queue_size": 10,
            "approx_sync": True,
            
            # Memory Management
            "Mem/IncrementalMemory": "true",
            "Mem/STMSize": "10",  # Short-term memory begrenzen
            "Mem/ImagePreDecimation": "2",  # Bilder vorher verkleinern (2x)
            "Mem/ImagePostDecimation": "1",
            
            # ============ LOOP CLOSURE OPTIMIERUNG ============
            # Weniger aggressive Loop Closure
            "RGBD/LoopClosureReextractFeatures": "false",  # Performance-Boost!
            "Kp/MaxFeatures": "200",  # weniger Features (default 400)
            "Kp/DetectorStrategy": "6",  # GFTT/Good Features (schneller als ORB)
            
            # Loop Closure Schwellwerte anpassen
            "Rtabmap/LoopThr": "0.15",  # höher = weniger false positives
            "RGBD/OptimizeMaxError": "1.0",
            
            # ============ ODOMETRY INTEGRATION ============
            "odom_topic": "/Odometry",
            "odom_tf_angular_variance": "0.001",  # FAST-LIO ist präzise
            "odom_tf_linear_variance": "0.001",
            
            # Vertraue FAST-LIO mehr als visueller Odometrie
            "Reg/Force3DoF": "false",
            "RGBD/Enabled": "true",
            
            # ============ GRAPH OPTIMIZATION ============
            "RGBD/OptimizeFromGraphEnd": "false",
            "Optimizer/Strategy": "0",  # TORO (schneller als g2o)
            "Optimizer/Iterations": "10",  # weniger Iterationen
            
            # ============ TF PUBLISHING ============
            "publish_tf": True,
            "publish_map_tf": True,
            "tf_delay": "0.05",  # 50ms delay
            
            # ============ FEATURE DETECTION ============
            "Kp/MaxDepth": "4.0",  # nur Features bis 4m
            "Kp/MinDepth": "0.3",  # mindestens 30cm
            "Vis/MinInliers": "10",  # weniger streng
            "Vis/MaxDepth": "4.0",
            
            # ============ MAPPING ============
            "Grid/Sensor": "0",  # 0=camera, 1=laser
            "Grid/RangeMax": "5.0",
            "Grid/CellSize": "0.05",
        }],
        remappings=[
            ("rgbd_image", "/rtabmap/rgbd_image"),
            ("odom", "/Odometry"),
        ],
        arguments=[
            "-d",  # DB in ~/.ros/rtabmap.db
            # "--delete_db_on_start"  # Auskommentiert: nur zum Testen aktivieren
        ]
    )

    # 3) RTABMAP Visualisierung (optional, kann Performance kosten!)
    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "frame_id": "body",
            "odom_frame_id": "camera_init",
            "subscribe_rgbd": True,
            "subscribe_odom_info": True,
            "queue_size": 10,
        }],
        remappings=[
            ("rgbd_image", "/rtabmap/rgbd_image"),
            ("odom", "/Odometry"),
        ]
    )

    return LaunchDescription([
        rgbd_sync,
        rtabmap,
        # rtabmap_viz  # Auskommentiert für bessere Performance - nur bei Bedarf aktivieren
    ])