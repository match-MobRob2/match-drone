# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## Project Overview

**Match Drone** ("Mächtige Match Möve Marvin") is a ROS2-based autonomous drone simulation system integrating PX4 SITL, Gazebo Harmonic, FAST-LIO2 SLAM, and GPU-accelerated Lidar (RGL). Designed for research and education at MATCH Institute.

**Tech Stack**: ROS2 Humble, PX4 SITL, Gazebo Harmonic, Ubuntu 24.04

**Key Documentation**:
- [misc/PROJECT.md](misc/PROJECT.md) - Architecture & module descriptions
- [misc/STATE.md](misc/STATE.md) - Implementation status & gaps
- [README.md](README.md) - Installation guide

---

## Essential Commands

### Initial Setup

```bash
# Automated (Ubuntu 24.04 only)
cd misc/
./setup.sh

# Manual PX4 Setup (required for SITL)
cd PX4-Autopilot/
bash ./Tools/setup/ubuntu.sh
DONT_RUN=1 make px4_sitl gz_x500

# Custom Models Installation (required after model changes)
cd match_models/
./install_models.sh
# Note: Models are COPIED to ~/.gz/models/, not symlinked
# Re-run after editing match_models/sdf/* or match_models/worlds/*
```

### Build & Run

```bash
# Build workspace (from workspace root, not src/)
cd /path/to/ros2_ws/
colcon build --symlink-install

# Build specific package
colcon build --packages-select match_control

# Source environment
source install/setup.bash

# Launch main simulation (standard variant)
ros2 launch match_launch match_drohne_alles.launch.py

# Launch arguments
ros2 launch match_launch match_drohne_alles.launch.py \
  gz_model:=match_drohne_nogps \  # Drone variant
  gz_world:=garbsen \              # World file
  x:=-18.0 y:=0.0 z:=1.0           # Spawn position

# Launch SLAM (FAST-LIO2 + RTAB-Map)
ros2 launch match_launch rtabmap_rgbd_fastlio.launch.py
```

### Running Demos

```bash
# Demo flights (after launching simulation)
ros2 run match_control demo_takeoff_land
ros2 run match_control demo_takeoff_forward_land
ros2 run match_control demo_takeoff_square_land
ros2 run match_control demo_takeoff_circle_land
```

### Development Workflow

```bash
# After editing Python nodes in match_* packages
colcon build --symlink-install  # Only needed once (--symlink-install)
source install/setup.bash
# Python changes are now live (no rebuild needed)

# After editing SDF models
cd match_models/
./install_models.sh
# PX4 rebuild only needed if adding NEW drone variants:
cd ../PX4-Autopilot/
DONT_RUN=1 make px4_sitl gz_x500
```

---

## High-Level Architecture

### ROS2 Package Structure

```
match_* packages (core functionality):
├─ match_control    - Flight missions & control services
├─ match_launch     - Launch orchestration
├─ match_models     - Gazebo models & worlds (SDF files)
├─ match_slam       - FAST-LIO2 configuration
├─ match_utils      - Sensor preprocessing & bridges
├─ match_nav        - Navigation stack (SKELETON - needs implementation)
└─ match_ai         - AI/ML integration (PLACEHOLDER - empty)

External dependencies (git submodules):
├─ PX4-Autopilot    - SITL autopilot (has COLCON_IGNORE)
├─ FAST_LIO_ROS2    - Lidar-Inertial SLAM
├─ RGLGazeboPlugin  - GPU Lidar simulation
└─ livox_ros_driver2 - Livox Lidar ROS2 driver
```

### Data Flow Pipeline

```
Gazebo Simulator (PX4 SITL)
  ↓
Sensor Plugins:
  ├─ RGL Lidar → /rgl_lidar (PointCloud2)
  ├─ RealSense → /match_drohne_alles/front_depth/* (RGB + Depth)
  └─ IMU → /mavros/imu/data
  ↓
match_utils (preprocessing):
  ├─ imu_timemachine: Rewrites timestamps to sim time (CRITICAL for SLAM)
  ├─ cloud_nan_filter: Filters NaN, distance limits (0.5-30m), downsamples to 10k points
  ├─ pointcloud_to_livox: Converts RGL PointCloud2 → Livox CustomMsg format
  └─ odometry_to_drone: SLAM odometry → PX4 EKF2 format
  ↓
SLAM Pipeline:
  ├─ FAST-LIO2: Lidar + IMU → /Odometry + Map (no loop closure)
  └─ RTAB-Map: RGB-D + FAST-LIO2 odom → Loop closure + Dense map
  ↓
match_control:
  ├─ drone_services: distance_to_point, is_ready
  └─ demo_*: State machine-based flight missions
  ↓
MAVROS Bridge → PX4 SITL → Gazebo Actuators
```

### Critical Synchronization Points

1. **Simulation Time**: `imu_timemachine` node is REQUIRED for SLAM to work correctly
   - Without it: Timestamp mismatches cause SLAM failure
   - Subscribes to `/clock`, rewrites IMU timestamps

2. **Frame Conventions**:
   - PX4: FRD (Front-Right-Down) / NED (North-East-Down)
   - ROS2: FLU (Front-Left-Up) / ENU (East-North-Up)
   - MAVROS handles conversions automatically

3. **Odometry Fusion**:
   - `odometry_to_drone` feeds SLAM output into PX4 EKF2
   - Enables GPS-denied navigation with `match_drohne_nogps`

---

## Drone Model Variants

Located in `match_models/sdf/`:

| Model | Sensors | Use Case | PX4 Autostart |
|-------|---------|----------|---------------|
| `match_drohne_alles` | RGB + Depth + Lidar + IMU + GPS | Full SLAM | 40014 |
| `match_drohne_nogps` | RGB + Depth + Lidar + IMU (no GPS) | Indoor/GPS-denied | 40015 |
| `match_drohne_lidar` | Lidar + IMU only | Lidar-only SLAM | 40012 |
| `match_drohne_fr_camera` | RGB only | Visual-only tasks | 40013 |
| `match_drohne` | Base (no sensors) | Template | 40011 |

**Adding a new variant**:
1. Copy existing model in `match_models/sdf/`
2. Edit SDF file (add/remove sensors)
3. Create airframe file in `match_models/misc/400XX_gz_your_model`
4. Run `./install_models.sh`
5. Rebuild PX4: `cd PX4-Autopilot && DONT_RUN=1 make px4_sitl gz_x500`

---

## Common Development Patterns

### Adding a New Flight Mission

1. Create node in `match_control/match_control/your_mission.py`:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class YourMissionNode(Node):
    def __init__(self):
        super().__init__('your_mission_node')
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        # See demo_takeoff_land.py for full template
```

2. Add entry point in `match_control/setup.py`:
```python
'your_mission = match_control.your_mission:main',
```

3. Rebuild: `colcon build --packages-select match_control`

### Adjusting SLAM Parameters

Edit `match_slam/config/fast_lio2_params.yaml`:
```yaml
# Key parameters:
preprocess:
  blind: 0.15          # Minimum Lidar range (m)

mapping:
  fov_degree: 60       # Field of view (degrees)
  det_range: 20.0      # Detection range (m)
  cube_side_length: 1000.0  # Map size (m)

# Extrinsic calibration (Lidar → IMU):
extrinsic_T: [-0.011, -0.02329, 0.04412]  # Translation [x,y,z]
extrinsic_R: [1, 0, 0,                    # Rotation matrix
              0, 1, 0,
              0, 0, 1]
```

**After changes**: Just restart SLAM launch file (no rebuild needed for YAML)

---

## Important Constraints & Gotchas

### 1. Working Directory Matters
- **Always run from ROS2 workspace root**, NOT from `src/`
- PX4 launch uses relative path: `./PX4-Autopilot/build/...`
- If you get "PX4 binary not found", you're in the wrong directory

### 2. QGroundControl Required
- Simulation won't ARM without QGroundControl connected
- PX4 safety feature (even in SITL)
- Install: See [docs/QGroundControl.md](docs/QGroundControl.md)

### 3. Model Changes Don't Auto-Update
- Models are COPIED to `~/.gz/models/` by `install_models.sh`
- Editing `match_models/sdf/` directly has NO effect until re-running script
- Alternative: Edit `~/.gz/models/` directly for quick tests (not recommended)

### 4. Submodules Are Modified
- `git status` shows `m` (modified) for several submodules (DB-TSDF, FAST_LIO, etc.)
- This is expected (local patches/configs)
- Don't blindly `git submodule update --init` (will lose changes)

### 5. FAST-LIO2 Needs Unstructured Cloud
- Config has `scan_line: 1` (treats as single-line Lidar)
- Real Livox Mid-360 has structured data, but RGL simulation doesn't
- Don't change to `scan_line: 4` or higher (will break)

### 6. GPU Required for Real-Time
- RGLGazeboPlugin (Lidar) needs NVIDIA GPU for >1.0 real-time factor
- CPU fallback runs at ~0.3x real-time
- Simulation still works, just slower

---

## Testing & Validation

### Manual Smoke Test

```bash
# Terminal 1: Launch simulation
ros2 launch match_launch match_drohne_alles.launch.py

# Terminal 2: Check topics are publishing
ros2 topic hz /mavros/local_position/pose  # Should be ~30Hz
ros2 topic hz /rgl_lidar                   # Should be ~10Hz
ros2 topic hz /mavros/imu/data             # Should be ~100Hz

# Terminal 3: Run demo
ros2 run match_control demo_takeoff_land

# Expected: Drone arms, takes off to 2m, hovers 10s, lands
```

### SLAM Validation

```bash
# Terminal 1: Launch simulation
ros2 launch match_launch match_drohne_alles.launch.py

# Terminal 2: Launch SLAM
ros2 launch match_launch rtabmap_rgbd_fastlio.launch.py

# Terminal 3: Check SLAM output
ros2 topic hz /Odometry  # Should be ~10Hz
ros2 topic echo /Odometry --once  # Check if pose is reasonable

# Terminal 4: Visualize in RViz2
rviz2 -d match_slam/config/fast_lio2_config.rviz
# Add /Odometry (Odometry display), /map (PointCloud2)
```

### Known Test File

- `test.sh` exists but contains RTAB-Map launch (not automated test)
- No unit tests currently implemented (see misc/STATE.md for recommendations)

---

## Module Implementation Status

| Module | Status | Priority for Implementation |
|--------|--------|----------------------------|
| match_control | ✅ Functional | Low (complete) |
| match_launch | ✅ Functional | Low (complete) |
| match_models | ✅ Functional | Low (complete) |
| match_slam | ✅ Functional | Medium (add map save/load) |
| match_utils | ✅ Functional | Low (complete) |
| match_nav | ⚠️ Skeleton | **HIGH** (needs waypoint follower, collision avoidance) |
| match_ai | ❌ Empty | Medium (YOLO object detection, depth estimation) |

**Recommended next steps**: See [misc/STATE.md](misc/STATE.md) Section 13 for prioritized roadmap.

---

## Code Style & Conventions

### Observed Patterns

- **ROS2 Nodes**: Python-based (rclpy), state machine pattern for missions
- **Launch Files**: Python launch API (not XML)
- **Config**: YAML for parameters, SDF for models
- **Frame IDs**:
  - World: `map` (ENU)
  - Body: `base_link` (FLU)
  - Odometry: `odom` (ENU)
  - Sensors: `<sensor_name>_optical_frame`

### Commit Message Style

**Current**: Informal/humorous (e.g., "KABUUMMMMMMMMM", "liest das eigentlich jemand?")
**Recommendation**: Consider Conventional Commits for clarity (e.g., `feat:`, `fix:`, `docs:`)

---

## External Resources

- PX4 Docs: https://docs.px4.io/
- ROS2 Humble: https://docs.ros.org/en/humble/
- Gazebo Harmonic: https://gazebosim.org/
- FAST-LIO2 Paper: https://github.com/hku-mars/FAST_LIO

---

**Last Updated**: 2026-02-10 (via automated analysis)
**Maintainer**: Match-MobRob2 Team (Luca, Tobi)
