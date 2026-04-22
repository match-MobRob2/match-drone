ros2 launch rtabmap_launch rtabmap.launch.py     rtabmap_args:="--delete_db_on_start"     rgb_topic:=/zed/zed_node/rgb/image_rect_color     depth_topic:=/zed/zed_node/depth/depth_registered     camera_info_topic:=/zed/zed_node/rgb/camera_info     frame_id:=zed_camera_link     approx_sync:=false     wait_imu_to_init:=true     imu_topic:=mavros/imu/data     rviz:=true

