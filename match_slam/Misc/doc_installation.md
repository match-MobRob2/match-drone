# Compatibility and build configurations 

This project has been tested and verified to work with the following versions:

- **Python**: 3.8 - 3.11
- **Operating Systems**:
  - Ubuntu 24.04 LTS
- **ROS Distro**:
  - Jazzy


For best results, ensure your environment matches these specifications. Incompatible versions may lead to unexpected errors.

# Installation
Follow these steps to set up the project locally:

## Install general requirements for various packages through codebase

- Run following:
   ```bash
   pip install -r /path/to/requirements.txt
   ```

## Install ros-gz (ROS Jazzy)

  > [!TIP]
  > The `ros_gz` library makes heavy use of templates which causes compilers to consume a lot of memory. If your build fails with `c++: fatal error: Killed signal terminated program cc1plus`
  > try building with `colcon build --parallel-workers=1 --executor sequential`. You might also have to set `export MAKEFLAGS="-j 1"` before running `colcon build` to limit
  > the number of processors used to build a single package.
  
1. Run following:
   ```bash
   pip install -r /path/to/requirements.txt
   ```

## Install FAST-LIO2

1. Install livox-SDK
   ```bash
   git clone https://github.com/Livox-SDK/Livox-SDK2.git
   cd ./Livox-SDK2/
   mkdir build
   cd build
   cmake .. && make -j
   sudo make install
   ```
   - common issue when running "cmake .. && make -j" -> (SOLVED): Check @lukeliao's comment on Feb 25 https://github.com/Livox-SDK/Livox-SDK2/issues/90

2. Install Livox ROS Driver 2 
   ```bash
   git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
   ```
   Then for ROS Jazzy/Humble run:
   ```bash
   source /opt/ros/humble/setup.sh
   ./build.sh humble
   ```
3. Install FAST-LIO2 package 
   - Source the livox_ros_driver before build 
   ```bash
    cd <ros2_ws>/src 
    git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
    cd ..
    rosdep install --from-paths src --ignore-src -y
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select fast_lio --allow-overriding ros_gz_bridge ros_gz_image ros_gz_interfaces ros_gz_sim ros_gz_sim_demos
   ```

## Documentation
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