
✅ Done
---



### model.sdf (match_mpdels/sdf/match_drohne/model.sdf)
<ul>
  <li><input type="checkbox" checked disabled> Mount LiDAR + IMU sensors and their plugins </li>
  <li><input type="checkbox" checked disabled> Copy them to PX4 dir via ./install_models.sh</li>
  <li><input type="checkbox" checked disabled> Mount camera and its plugins</li>
  <li><input type="checkbox" checked disabled> Add tf + odom link</li>
  <li><input type="checkbox" checked disabled> Add gazebo's OdometryPublisher plugin </li>
  <li><input type="checkbox" checked disabled> Add gazebo's JointStatePublisher plugin </li>
</ul>

### robot_description 
<ul>
  <li><input type="checkbox" checked disabled> create robot_description file: drone.urdf.xacro (drone_core.xacro, lidar.xacro, camera.xacro) </li>
  <li><input type="checkbox" checked disabled> Load in livox.stl file </li>
  <li><input type="checkbox" checked disabled> Create robot_state_publisher launch file </li>
</ul>

### PX4 simulation bringup 
<ul>
  <li><input type="checkbox" checked disabled> Call robot_state_publisher node </li>
  <li><input type="checkbox" checked disabled> Call gazebo_server with correct params/args (PX4 and GZ paths are crucial) </li>
  <li><input type="checkbox" checked disabled> Setup ros_brigde correctly: Find topic names for LiDAR, IMU, Camera etc. and remap them if needed </li>
  <li><input type="checkbox" checked disabled> write extra code to fix PCL and IMU mismatch topic frame_id </li>
  <li><input type="checkbox" checked disabled> load px4 at the right time and with correct params/args:  "PX4_GZ_STANDALONE": "false", "PX4_SIMULATOR": "GZ" </li>
  <li><input type="checkbox" checked disabled> timing of nodes is crucial </li>
  <li><input type="checkbox" checked disabled> fix healthy issues (some sensors have to be init at start so that QGroundControl could arm drone) </li>
  <li><input type="checkbox" checked disabled> Set use_sim_time=True everywhere </li>
</ul>

### FAST-LIO2
<ul>
  <li><input type="checkbox" checked disabled> write extra code to tf_map_to_cam_init (bridge cam_init (fast-lio2's default output frame) to map with static transform) </li>
  <li><input type="checkbox" checked disabled> write extra code to odom_bridge </li>
  <li><input type="checkbox" checked disabled> adding static transforms </li>
  <li><input type="checkbox" checked disabled> Topic calibration (via fast-lio2_params.yaml) + sync (already taken care of by fast-lio2 laserMapping.cpp) </li>
  <li><input type="checkbox" checked disabled> call fast-lio2 with cusom fast-lio2_params.yaml </li>
  <li><input type="checkbox" checked disabled> test with classic control node </li>
  <li><input type="checkbox" checked disabled> Operate drone via keyboard telep + custom control node (check match_control package) </li>
</ul>

### Codebase
<ul>
  <li><input type="checkbox" checked disabled> document codebase </li>
  <li><input type="checkbox" checked disabled> add .sh scripts for full and partial setups </li>
  <li><input type="checkbox" checked disabled> add pip dependencies under requirements.txt  </li>
</ul>


⏳🔜 Open
---
<ul>
  <li><input type="checkbox" disabled> Add used ressources + useful links to repo</li>
  <li><input type="checkbox" disabled> Evaluate SLAM performance: compare Gazebo Odom to Fast-lio2's odometry then plot them in 3D</li>
  <li><input type="checkbox" disabled> Use evo tool (https://github.com/MichaelGrupp/evo)</li>
  <li><input type="checkbox" disabled> Use metrics like: ATE (Absolute Trajectory Error), RPE (Relative Pose Error)</li>
  <li><input type="checkbox" disabled> Lokalisierung (Use "pre-loading" Mode in fast-lio2)</li>
  <li><input type="checkbox" disabled> Navigation</li>
</ul>