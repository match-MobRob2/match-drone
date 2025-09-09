-------------------------
WIRING FAST-LIO2
-------------------------

(Terminal1) ros2 launch match_slam slam_fast_lio2.launch.py
(Terminal2) ros2 run fast_lio fastlio_mapping --ros-args --params-file src/match-drone/match_slam/config/fast_lio2_params.yaml --log-level fast_lio:=debug
/or/
ros2 launch fast_lio mapping.launch.py config_path:=/home/daghbeji/match_ws/src/match-drone/match_slam/config config_file:=fast_lio2_params.yaml rviz:=false


(Terminal3) ros2 run teleop_twist_keyboard teleop_twist_keyboard
(Terminal4) ros2 run match_control teleop_driven_flight
-------------------- Optional --------------------
(Terminal5) ros2 run rqt_tf_tree rqt_tf_tree
--------------------------------------------------

-------------------- Saving recorded map --------------------
(New terminal) ros2 service call /map_save std_srvs/srv/Trigger
--------------------------------------------------

-------------------- View recorded map --------------------
pcl_viewer recorded_map1.pcd
--------------------------------------------------

--------------------- Misc. ----------------------
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped.twist.linear.z
--------------------------------------------------