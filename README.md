🚀 Launch Simulation + Localization
-----

1. Run QGroundControl 
   ```bash
   ./QGroundControl-x86_64.AppImage
   ```

2. Launch sim node (in new terminal)
   ```bash
   ros2 launch match_slam localization_sim.launch.cpp
   ```

3. Run fast-lio2 package using custom YAML params file to start mapping (in new terminal)
   ```bash
   ros2 run fast_lio fastlio_mapping --ros-args --params-file src/match-drone/match_slam/config/fast_lio2_params.yaml --log-level fast_lio:=debug
   ```
4. Run localizer node (in new terminal)
   ```bash
   ros2 launch localizer localizer_launch.py
   ```

5. Run Relocalize service with custom recorded .pcd map (in new terminal)
   ```bash
   ros2 service call /localizer/relocalize interface/srv/Relocalize "{"pcd_path": "~/slam_ws/src/match-drone/match_slam/PCD_recorded/ndense_ascii_cleaned.pcd", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}"
   ```

5. Check if relocalization was successfull (in new terminal)
   ```bash
   ros2 service call /localizer/relocalize_check interface/srv/IsValid "{"code": 0}"
   ```
   
7. (Optional) Visualize TF Tree (in new terminal)
   ```bash
   ros2 run rqt_tf_tree rqt_tf_tree
   ```

⚙️ Requirements
---

pcl
Eigen
sophus
gtsam
livox_ros_driver2

1. PCL + Eigen:
   ```bash
   sudo apt update
   sudo apt install -y build-essential cmake git \
      libpcl-dev libeigen3-dev \
      ros-jazzy-pcl-conversions ros-jazzy-pcl-msgs \
      ros-jazzy-tf2-eigen ros-jazzy-geometry2
   sudo apt install libmetis-dev
   ```
2. Sophus:
   ```bash
   cd ~/slam_ws/src/
   git clone https://github.com/strasdat/Sophus.git
   cd Sophus
   git checkout 1.22.10
   mkdir build && cd build
   cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
   make
   sudo make install
   ```
   - If u encounter a build error, do this: add `add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)` to CMakeLists.txt

3. GTSAM (≥4.1.1):
   ```bash
   cd ~/slam_ws/src/
   git clone https://github.com/borglab/gtsam.git
   cd gtsam
   git checkout 4.1.1
   mkdir build && cd build
   cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
   make -j$(nproc)
   sudo make install
   ```