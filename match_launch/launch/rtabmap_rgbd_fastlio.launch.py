# rtabmap_rgbd_fastlio.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = True

    # 1) Synchronisiere RGB + Depth -> RGBDImage
    rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "approx_sync": True,
            "queue_size": 30
        }],
        remappings=[
            ("rgb/image",       "/match_drohne_alles/front_rgb/image"),
            ("rgb/camera_info", "/match_drohne_alles/front_rgb/camera_info"),
            ("depth/image",     "/match_drohne_alles/front_depth/image"),
            ("depth/camera_info","/match_drohne_alles/front_depth/camera_info"),
            ("rgbd_image",      "/rtabmap/rgbd_image"),
        ]
    )

    # 2) RTAB-Map SLAM Node
    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,

            # Frames
            "frame_id": "body",
            "odom_frame_id": "camera_init",
            "map_frame_id": "map",

            # Loop closure aktiv
            "Mem/IncrementalMemory": "true",
            "RGBD/LoopClosureReextractFeatures": "true",

            # Odom vom FAST-LIO (Topic)
            "odom_topic": "/Odometry",

            # RGBD input
            "subscribe_rgbd": True,
            "subscribe_depth": False,   # weil wir RGBDImage nutzen
            "subscribe_rgb": False,
            "subscribe_scan": False,

            # Praktische Defaults
            "Rtabmap/DetectionRate": "10.0",
            "queue_size": 30,
            "approx_sync": True,

            # Map->Odom TF publishen (wichtig!)
            "publish_tf": True,
            "publish_map_tf": True,
        }],
        remappings=[
            ("rgbd_image", "/rtabmap/rgbd_image"),
            ("odom", "/Odometry"),
        ],
        arguments=[
            "-d"  # DB in ~/.ros/rtabmap.db (für persistente Tests)
        ]
    )

    # 3) Optional: RTAB-Map Visualisierung (nur wenn gewünscht)
    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("rgbd_image", "/rtabmap/rgbd_image"),
            ("odom", "/Odometry"),
        ]
    )

    return LaunchDescription([rgbd_sync, rtabmap, rtabmap_viz])