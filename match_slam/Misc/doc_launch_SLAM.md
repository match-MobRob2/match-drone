# Bringup Simulation with FAST-LIO2 SLAM algorithm

## Launch commands

1. Bring up simulation
   ```bash
   ros2 launch match_slam slam_fast_lio2.launch.py
   ```

2. Operate drone via keyboard (in new terminal)
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

3. Run this custom code to send velocities to PX4 via MAVROS (in new terminal)
   ```bash
   ros2 run match_control teleop_driven_flight
   ```
   - Then check if vehicule is "ARMED" and on mode "OFFBOARD" by listening to following topic:
   ```bash
   ros2 topic echo /mavros/state
   ```
   - If for some reason the vehicule's mode is not "OFFBOARD" anymore, set it manually using this command:
   ```bash
   ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
   ```
   - If both are set, go back to "teleop_twist_keyboard" terminal and operate vehicule with keyboared.
   - For vehicule control check this documentation out: [teleop_docs.py](../../match_control/match_control/)

4. Run fast-lio2 package using custom YAML file to start mapping (in new terminal)
   ```bash
   ros2 run fast_lio fastlio_mapping --ros-args --params-file src/match-drone/match_slam/config/fast_lio2_params.yaml --log-level fast_lio:=debug
   ```
   or this:
   ```bash
   ros2 launch fast_lio mapping.launch.py config_path:=/home/daghbeji/match_ws/src/match-drone/match_slam/config config_file:=fast_lio2_params.yaml rviz:=false
   ```


5. Record map manually (in new terminal)
   ```bash
   ros2 service call /map_save std_srvs/srv/Trigger
   ```
   - Set "pcd_save_en: true" and "interval: 5-10" in fast_lio2_params.yaml
   - Modify the recorded map name in fast_lio2_params.yaml via parameter "map_file_path"
   - Recorded map has .pcd format

6. (Optional) Visualize TF Tree (in new terminal)
   ```bash
   ros2 run rqt_tf_tree rqt_tf_tree
   ```

## Ressourcen
- Startercode: [exercise_takeoff_circle_land.py](../../match_control/match_control/exercise_takeoff_circle_land.py)
- Musterlösung: [demo_takeoff_circle_land.py](../../match_control/match_control/demo_takeoff_circle_land.py)


