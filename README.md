This branch was established to integrate, test, and benchmark state-of-the-art SLAM (Self Localization and Mapping) algorithms for drones in simulation with the goal of mapping an indoor production environment.

System Architecture
-----
![alt text](system_architecture.png)

Software Requirements
-----

* **Ubuntu 24.04 LTS Noble**
* **ROS2 Jazzy** 
* **Gazebo Harmonic**
* **ROS_GZ**
* **PX4 firmware** installation on Linux: Autopilot software which includes the software-in-the-loop firmware
* **MAVROS** package: Autopilot ROS interface
* **QGroundControl** 

* **teleop_twist_keyboard** package: Teleoperation ROS interface
Check out this documentation for keyboard control (/keyboard_docs.md)
* **SLAM package** 

Installation Guide
-----
### For a one-shot installation, please launch this bash script
```bash
    cd misc/
    ./setup_entire_workspace.sh
```
### For a step-by-step installation, follow the following commands:

1. First of all clone the repository to your local directory:
```bash
    cd /path/to/your/ros2_ws/src
    git clone https://github.com/match-MobRob2/match-drone/ .    
```

2. For installing ROS2 jazzy, PX4, MAVROS, QGroundControl and load custom models to PX4 use this script at () or a step-by-step installation guide on the main branch of the repository (/main/READNE.md):
```bash
    cd misc/
    ./setup.sh  
```
- Everytime you modify the model.sdf under match_models, do the following to let PX4 know about the modifications:
```bash
    cd ~/ros2_ws/src/match_models/
    ./install_models.sh
```

3. Install gazebo harmonic (https://gazebosim.org/docs/harmonic/install_ubuntu/):
```bash
    cd misc/
    ./gazebo_harmonic_setup.sh  
```
- To test if gazebo works launch "gz sim".
- To check gazebo version, run "echo GZ_VERSION". If it is not harmonic, set it manually: "export GZ_VERSION=harmonic".

4. Install and compile ros-gz from source (https://github.com/gazebosim/ros_gz/tree/jazzy?tab=readme-ov-file):
```bash
    cd misc/
    ./ros_gz_setup.sh  
```

5. Install FAST-LIO2
```bash
    cd misc/
    ./fast_lio2_setup.sh  
```
- common issue while running "cmake .. && make -j" -> (SOLVED): Check out @lukeliao's comment on Feb 25: https://github.com/Livox-SDK/Livox-SDK2/issues/90

6. Install required general packages for various nodes throughout codebase
```bash
   cd misc/
   pip install -r requirements.txt
```

Bringup Simulation 
-----

1. Launch sim node
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

5. To save recorded map manually (in new terminal)
```bash
   ros2 service call /map_save std_srvs/srv/Trigger
```
   - Set "pcd_save_en: true" and "interval: 5-10" in fast_lio2_params.yaml
   - Modify the recorded map name in fast_lio2_params.yaml via parameter "map_file_path"
   - Recorded map has .pcd format
   - Check out FAST-LIO2 parameters documentation: [fast_li2_params_docs.py](../docs/match_control/)
   - To visualize recorded map use pcl_viewer: 
```bash
   pcl_viewer recorded_map1.pcd
```

6. (Optional) Visualize TF Tree (in new terminal)
```bash
   ros2 run rqt_tf_tree rqt_tf_tree
```

Documentation
-----
The fast-lio2 SLAM algorithm originates from the following paper:
```
@article{DBLP:journals/corr/abs-2107-06829,
  author       = {Wei Xu and
                  Yixi Cai and
                  Dongjiao He and
                  Jiarong Lin and
                  Fu Zhang},
  title        = {{FAST-LIO2:} Fast Direct LiDAR-inertial Odometry},
  journal      = {CoRR},
  volume       = {abs/2107.06829},
  year         = {2021},
  url          = {https://arxiv.org/abs/2107.06829},
  eprinttype    = {arXiv},
  eprint       = {2107.06829},
  timestamp    = {Fri, 21 Jun 2024 12:54:52 +0200},
  biburl       = {https://dblp.org/rec/journals/corr/abs-2107-06829.bib},
  bibsource    = {dblp computer science bibliography, https://dblp.org}
}
```