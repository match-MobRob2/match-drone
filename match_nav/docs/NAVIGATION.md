# Match Nav - 3D Navigation Stack Documentation

**Version**: 0.0.1 (Phase 1 MVP)
**Date**: 2026-02-10

---

## Overview

**match_nav** is a complete 3D navigation stack for the Match Drone project, enabling:
- **3D Path Planning** with OMPL (MVP: simplified planning)
- **Waypoint Following** with state machine control
- **Collision Avoidance** (Phase 2)
- **Autonomous Exploration** with frontier-based mapping (Phase 3)

### Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    MISSION MANAGER                       │
│         (High-Level State Machine & Orchestration)       │
└────────────────┬────────────────────────┬────────────────┘
                 │                        │
         ┌───────▼──────────┐    ┌────────▼───────────┐
         │  GLOBAL PLANNER  │    │ WAYPOINT FOLLOWER  │
         │  (Path Planning) │    │  (Trajectory Exec) │
         └──────────────────┘    └────────────────────┘
                 ▲                         │
                 │                         ▼
         ┌───────┴──────────┐    ┌────────────────────┐
         │   OCTOMAP SERVER │    │  MAVROS (PX4 SITL) │
         │   (3D Mapping)   │    │  (Drone Control)   │
         └──────────────────┘    └────────────────────┘
                 ▲
                 │
         ┌───────┴──────────┐
         │   FAST-LIO2 SLAM │
         │   (Point Cloud)  │
         └──────────────────┘
```

---

## Installation

### 1. Dependencies

**ROS2 Packages** (install via apt):
```bash
# Required
sudo apt install ros-humble-nav-msgs ros-humble-geometry-msgs ros-humble-sensor-msgs

# SLAM (should already be installed)
# - FAST-LIO2 (in workspace)
# - RTAB-Map (via match_launch)

# Optional (Phase 2+)
sudo apt install ros-humble-octomap-server ros-humble-octomap-msgs
sudo apt install ros-humble-ompl  # For OMPL path planning
```

**Python Dependencies** (should be available in ROS2):
```bash
pip3 install numpy
```

### 2. Build Package

```bash
cd /path/to/ros2_ws/
colcon build --packages-select match_nav --symlink-install
source install/setup.bash
```

**Verify Build**:
```bash
ros2 pkg list | grep match_nav
# Should output: match_nav

ros2 interface list | grep match_nav
# Should show custom messages/services
```

---

## Quick Start

### 1. Launch Navigation Stack

```bash
# Terminal 1: Launch navigation (includes SLAM stack via mapper.launch.py)
ros2 launch match_nav navigation.launch.py
```

This starts:
- Gazebo simulation (via mapper.launch.py)
- Drohne + PX4 SITL (via mapper.launch.py)
- FAST-LIO2 + RTAB-Map SLAM (via mapper.launch.py)
- Global Planner Node
- Waypoint Follower Node
- Mission Manager Node

### 2. Arm and Takeoff (Using match_control)

```bash
# Terminal 2: Takeoff to 2m
ros2 run match_control demo_takeoff_land
```

### 3. Navigate to Pose

```bash
# Terminal 3: Send navigation command
ros2 service call /match_nav/navigate_to_pose match_nav/srv/NavigateToPose \
  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 5.0, z: 2.0}, orientation: {w: 1.0}}}}"
```

Expected behavior:
1. Global Planner plans path from current position to (5, 5, 2)
2. Mission Manager publishes path to Waypoint Follower
3. Waypoint Follower follows path, publishing setpoints to MAVROS
4. Drone flies to goal

---

## Nodes

### 1. Global Planner (`global_planner`)

**Service**: `/match_nav/plan_path`

Plans collision-free 3D paths from start to goal.

**Phase 1 (MVP)**: Simple straight-line interpolation
**Phase 2**: OMPL RRT* with Octomap collision checking

**Parameters** (`config/planner.yaml`):
- `planner_type`: Algorithm (default: "RRTstar")
- `planning_time`: Timeout (default: 5.0 seconds)
- `waypoint_spacing`: Distance between waypoints (default: 1.0m)
- `bounds.min_z` / `max_z`: Flight altitude limits (default: 0.5m - 20.0m)

**Example Call**:
```bash
ros2 service call /match_nav/plan_path match_nav/srv/PlanPath \
  "{start: {position: {x: 0.0, y: 0.0, z: 2.0}},
    goal: {position: {x: 10.0, y: 10.0, z: 2.0}},
    planning_time: 5.0}"
```

---

### 2. Waypoint Follower (`waypoint_follower`)

**Topic** (Subscription): `/match_nav/mission_path` (nav_msgs/Path)
**Topic** (Publication): `/mavros/setpoint_position/local` (PoseStamped)
**Services**:
- `/match_nav/start_mission` (std_srvs/Trigger)
- `/match_nav/abort_mission` (std_srvs/Trigger)
- `/match_nav/pause_mission` (std_srvs/Trigger)

Follows waypoints sequentially with state machine control.

**States**:
- `IDLE (0)`: Waiting for mission
- `NAVIGATING (1)`: Following waypoints
- `AVOIDING (2)`: Obstacle detected (Phase 2)
- `REACHED_GOAL (3)`: Mission completed
- `ERROR (4)`: Error state

**Parameters** (`config/navigation.yaml`):
- `control_frequency`: Control loop rate (default: 10.0 Hz)
- `waypoint_threshold`: Distance to waypoint (default: 0.3m)
- `max_velocity`: Max speed (default: 2.0 m/s)
- `waypoint_timeout`: Timeout per waypoint (default: 30.0s)

**Status Topic**: `/match_nav/status` (NavigationStatus)

---

### 3. Mission Manager (`mission_manager`)

**Service**: `/match_nav/navigate_to_pose` (NavigateToPose)

High-level orchestration node coordinating planning and execution.

**Workflow**:
1. Receive navigate_to_pose request
2. IDLE → PLANNING state
3. Call global_planner to plan path
4. Publish path to waypoint_follower
5. Call start_mission on waypoint_follower
6. PLANNING → NAVIGATING state
7. Monitor execution via status topic
8. NAVIGATING → COMPLETED when goal reached

**Parameters** (`config/navigation.yaml`):
- `replan_on_failure`: Auto-replan on error (default: true)
- `max_replan_attempts`: Max replans (default: 3)
- `execution_timeout`: Mission timeout (default: 120.0s)

**Cancel Mission**:
```bash
ros2 service call /match_nav/cancel_mission std_srvs/srv/Trigger
```

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/match_nav/mission_path` | nav_msgs/Path | Current mission waypoints |
| `/match_nav/status` | NavigationStatus | Waypoint follower status |
| `/mavros/setpoint_position/local` | PoseStamped | Setpoints to PX4 |
| `/mavros/local_position/pose` | PoseStamped | Current drone pose |
| `/cloud_registered` | PointCloud2 | FAST-LIO2 point cloud |
| `/octomap_binary` | Octomap | 3D occupancy map (Phase 2) |

---

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/match_nav/navigate_to_pose` | NavigateToPose | **Main entry point** for navigation |
| `/match_nav/plan_path` | PlanPath | Global path planning |
| `/match_nav/start_mission` | Trigger | Start waypoint following |
| `/match_nav/abort_mission` | Trigger | Abort current mission |
| `/match_nav/pause_mission` | Trigger | Pause/resume mission |
| `/match_nav/cancel_mission` | Trigger | Cancel via mission manager |

---

## Configuration

### Planner Configuration (`config/planner.yaml`)

```yaml
global_planner:
  ros__parameters:
    planner_type: "RRTstar"
    planning_time: 5.0
    waypoint_spacing: 1.0
    bounds:
      min_x: -100.0
      max_x: 100.0
      min_y: -100.0
      max_y: 100.0
      min_z: 0.5    # Min flight height
      max_z: 20.0   # Max flight height
```

### Navigation Configuration (`config/navigation.yaml`)

```yaml
waypoint_follower:
  ros__parameters:
    control_frequency: 10.0
    waypoint_threshold: 0.3  # meters
    max_velocity: 2.0        # m/s
    collision_check_enabled: true

mission_manager:
  ros__parameters:
    replan_on_failure: true
    max_replan_attempts: 3
    execution_timeout: 120.0  # seconds
```

---

## Usage Examples

### Example 1: Simple Navigation

```bash
# 1. Launch navigation stack
ros2 launch match_nav navigation.launch.py

# 2. Arm and takeoff (separate terminal)
ros2 run match_control demo_takeoff_land

# 3. Navigate to (10, 10, 2)
ros2 service call /match_nav/navigate_to_pose match_nav/srv/NavigateToPose \
  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 10.0, z: 2.0}, orientation: {w: 1.0}}}}"

# 4. Monitor status
ros2 topic echo /match_nav/status
```

### Example 2: Multi-Waypoint Mission (Manual)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

class MultiWaypointMission(Node):
    def __init__(self):
        super().__init__('multi_waypoint_mission')
        self.path_pub = self.create_publisher(Path, '/match_nav/mission_path', 10)
        self.start_client = self.create_client(Trigger, '/match_nav/start_mission')

    def send_mission(self):
        # Create path with multiple waypoints
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        waypoints = [
            (5.0, 0.0, 2.0),
            (5.0, 5.0, 2.0),
            (0.0, 5.0, 2.0),
            (0.0, 0.0, 2.0),
        ]

        for (x, y, z) in waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        # Publish path
        self.path_pub.publish(path)
        self.get_logger().info(f'Published path with {len(path.poses)} waypoints')

        # Start mission
        request = Trigger.Request()
        future = self.start_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'Start mission: {response.message}')

def main():
    rclpy.init()
    node = MultiWaypointMission()
    node.send_mission()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Save as `multi_waypoint_mission.py` and run:
```bash
python3 multi_waypoint_mission.py
```

---

## Troubleshooting

### Issue: Navigation nodes not starting

**Check**:
```bash
ros2 node list | grep match_nav
# Should show: global_planner, waypoint_follower, mission_manager
```

**Solution**: Ensure colcon build succeeded and sourced install/setup.bash

---

### Issue: "No mission path available"

**Symptoms**: start_mission service returns error

**Cause**: No path published to `/match_nav/mission_path`

**Solution**:
1. Use mission_manager's navigate_to_pose service (recommended)
2. OR manually publish path before calling start_mission

---

### Issue: Drone not moving

**Check**:
1. Is drone armed? `ros2 topic echo /mavros/state`
2. Is OFFBOARD mode active?
3. Are setpoints being published? `ros2 topic hz /mavros/setpoint_position/local`

**Solution**:
- Use match_control demo to arm and switch to OFFBOARD first
- Ensure QGroundControl is connected

---

### Issue: Planning fails

**Check**:
```bash
ros2 topic echo /match_nav/status
# Look for status_message with error details
```

**Common causes**:
- Goal out of bounds (check planner config bounds)
- Planning timeout too short
- No pose available (wait for SLAM to initialize)

---

## Roadmap

### Phase 1: MVP ✅ (Current)
- ✅ Waypoint Follower
- ✅ Simple Path Planning (straight line)
- ✅ Mission Manager
- ✅ Launch integration

### Phase 2: Collision Avoidance (Next)
- ⏳ Octomap Server integration
- ⏳ OMPL RRT* with collision checking
- ⏳ Collision Checker node
- ⏳ Dynamic obstacle avoidance

### Phase 3: Autonomous Exploration
- ⏳ Frontier Explorer node
- ⏳ Information gain evaluation
- ⏳ exploration.launch.py
- ⏳ Map persistence

### Phase 4: Optimization
- ⏳ Path smoothing
- ⏳ Velocity profiling
- ⏳ Multi-goal planning
- ⏳ Performance tuning

---

## API Reference

### Custom Messages

**NavigationStatus** (`match_nav/msg/NavigationStatus.msg`):
```
uint8 IDLE=0
uint8 NAVIGATING=1
uint8 AVOIDING=2
uint8 REACHED_GOAL=3
uint8 ERROR=4

uint8 state
float32 distance_to_goal
uint32 current_waypoint_index
uint32 total_waypoints
string status_message
```

### Custom Services

**NavigateToPose** (`match_nav/srv/NavigateToPose.srv`):
```
geometry_msgs/PoseStamped goal
---
bool success
string message
```

**PlanPath** (`match_nav/srv/PlanPath.srv`):
```
geometry_msgs/Pose start
geometry_msgs/Pose goal
float32 planning_time
---
nav_msgs/Path path
bool success
string error_message
```

---

## Contributing

See [../CLAUDE.md](../../CLAUDE.md) for development guidelines.

**Testing Checklist**:
- [ ] Build succeeds without errors
- [ ] All nodes start without errors
- [ ] Services respond correctly
- [ ] Navigation to known goal works
- [ ] Drone follows path smoothly
- [ ] Mission can be cancelled

---

## Support

- **Issues**: [GitHub Issues](https://github.com/match-MobRob2/match-drone/issues)
- **Documentation**: [Main README](../../README.md)
- **Architecture**: [misc/PROJECT.md](../../misc/PROJECT.md)

---

**Last Updated**: 2026-02-10
**Maintainer**: Match Drone Team (Luca, Tobi)
