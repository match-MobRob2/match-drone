# match Drone - Autonomous UAV Simulation Platform

<p align="center">
  <img src="docs/img/marvin.jpg" width="45%" />
  <img src="docs/img/marvin2.jpg" width="45%" />
</p>

<p align="center">
  <img alt="Ubuntu" src="https://img.shields.io/badge/Ubuntu-22.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white">
  <img alt="ROS2" src="https://img.shields.io/badge/ROS%202-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white">
  <img alt="PX4" src="https://img.shields.io/badge/PX4-SITL-000000?style=for-the-badge&logo=px4&logoColor=white">
  <img alt="Gazebo" src="https://img.shields.io/badge/Gazebo-Harmonic-6A5ACD?style=for-the-badge">
</p>

---

## Overview

ROS2-based autonomous UAV simulation system integrating PX4 SITL, Gazebo Harmonic, FAST-LIO2 SLAM, and GPU-accelerated Lidar (RGL) for autonomous flight research.

**Key Components:**
- FAST-LIO2 Lidar-Inertial SLAM
- RTAB-Map RGB-D SLAM with loop closure
- PX4 SITL autopilot
- Multiple sensor configurations (Lidar, RGB-D, IMU, GPS)

**Tech Stack:** ROS2 Humble | PX4 SITL | Gazebo Harmonic | Ubuntu 22.04

---

## Architecture


### Package Structure

```
marvin_* (Core Packages)
├── marvin_control       - Flight missions, state machines, services
├── marvin_launch        - Launch file orchestration
├── marvin_models        - Gazebo SDF models, worlds, airframes
├── marvin_slam          - FAST-LIO2 configuration
├── marvin_utils         - Sensor preprocessing, timestamp sync
├── marvin_nav           - Navigation stack (waypoint follower, planner)
└── marvin_ai            - AI/ML integration (object detection, depth)

External Dependencies (Git Submodules)
├── PX4-Autopilot       - Autopilot simulation
├── FAST_LIO_ROS2       - Lidar-Inertial SLAM
├── RGLGazeboPlugin     - GPU-accelerated Lidar
├── livox_ros_driver2   - Livox message format support
├── DB-TSDF             - Dense mapping (optional)
└── Livox-SDK2          - Livox sensor SDK
```

---

<img src="docs/img/grrr.png" width="300">
