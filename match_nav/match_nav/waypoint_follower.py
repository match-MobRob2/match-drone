#!/usr/bin/env python3
"""
Waypoint Follower Node for Match Drone Navigation Stack

Subscribes to mission paths and follows waypoints sequentially,
publishing setpoints to MAVROS for PX4 control.

Author: Match Drone Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import numpy as np

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from match_nav_msgs.msg import NavigationStatus


class WaypointFollower(Node):
    """
    Waypoint Follower Node

    State Machine:
    - IDLE: Waiting for mission
    - NAVIGATING: Following waypoints
    - AVOIDING: Obstacle detected, paused
    - REACHED_GOAL: Mission completed
    - ERROR: Error state
    """

    # State constants (match NavigationStatus.msg)
    IDLE = 0
    NAVIGATING = 1
    AVOIDING = 2
    REACHED_GOAL = 3
    ERROR = 4

    def __init__(self):
        super().__init__('waypoint_follower')

        # Declare parameters
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('waypoint_threshold', 0.3)
        self.declare_parameter('yaw_threshold', 0.1)
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('max_vertical_velocity', 1.0)
        self.declare_parameter('collision_check_distance', 2.0)
        self.declare_parameter('collision_check_enabled', True)
        self.declare_parameter('waypoint_timeout', 30.0)
        self.declare_parameter('status_publish_rate', 1.0)

        # Get parameters
        self.control_freq = self.get_parameter('control_frequency').value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.yaw_threshold = self.get_parameter('yaw_threshold').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.collision_check_dist = self.get_parameter('collision_check_distance').value
        self.collision_check_enabled = self.get_parameter('collision_check_enabled').value
        self.waypoint_timeout = self.get_parameter('waypoint_timeout').value
        self.status_rate = self.get_parameter('status_publish_rate').value

        # State variables
        self.state = self.IDLE
        self.current_pose = None
        self.mission_path = None
        self.current_waypoint_idx = 0
        self.waypoint_start_time = None
        self.status_message = "Initialized"

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10
        )

        self.mission_path_sub = self.create_subscription(
            Path,
            '/match_nav/mission_path',
            self.mission_path_callback,
            10
        )

        # Optional: Collision detection (Phase 2)
        if self.collision_check_enabled:
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            self.pointcloud_sub = self.create_subscription(
                PointCloud2,
                '/front_depth/points_filtered',
                self.pointcloud_callback,
                qos_profile
            )
            self.obstacle_detected = False

        # Publishers
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        self.status_pub = self.create_publisher(
            NavigationStatus,
            '/match_nav/status',
            10
        )

        # Services
        self.start_srv = self.create_service(
            Trigger,
            '/match_nav/start_mission',
            self.start_mission_callback
        )

        self.abort_srv = self.create_service(
            Trigger,
            '/match_nav/abort_mission',
            self.abort_mission_callback
        )

        self.pause_srv = self.create_service(
            Trigger,
            '/match_nav/pause_mission',
            self.pause_mission_callback
        )

        # Timers
        self.control_timer = self.create_timer(
            1.0 / self.control_freq,
            self.control_loop
        )

        self.status_timer = self.create_timer(
            1.0 / self.status_rate,
            self.publish_status
        )

        self.get_logger().info('Waypoint Follower initialized')

    def pose_callback(self, msg):
        """Update current pose"""
        self.current_pose = msg

    def mission_path_callback(self, msg):
        """Receive new mission path"""
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty mission path')
            return

        self.mission_path = msg
        self.get_logger().info(f'Received mission path with {len(msg.poses)} waypoints')

    def pointcloud_callback(self, msg):
        """
        Check for obstacles in pointcloud (simple implementation)
        Full implementation in Phase 2 with collision_checker node
        """
        # TODO: Implement proper collision detection
        # For now, just placeholder
        self.obstacle_detected = False

    def start_mission_callback(self, request, response):
        """Start mission service callback"""
        if self.mission_path is None:
            response.success = False
            response.message = "No mission path available"
            return response

        if self.current_pose is None:
            response.success = False
            response.message = "No pose available"
            return response

        self.current_waypoint_idx = 0
        self.waypoint_start_time = self.get_clock().now()
        self.state = self.NAVIGATING
        self.status_message = "Mission started"

        response.success = True
        response.message = f"Started mission with {len(self.mission_path.poses)} waypoints"
        self.get_logger().info(response.message)

        return response

    def abort_mission_callback(self, request, response):
        """Abort mission service callback"""
        self.state = self.IDLE
        self.mission_path = None
        self.current_waypoint_idx = 0
        self.status_message = "Mission aborted"

        response.success = True
        response.message = "Mission aborted"
        self.get_logger().info(response.message)

        return response

    def pause_mission_callback(self, request, response):
        """Pause mission service callback"""
        if self.state == self.NAVIGATING:
            self.state = self.AVOIDING
            self.status_message = "Mission paused"
            response.success = True
            response.message = "Mission paused"
        elif self.state == self.AVOIDING:
            self.state = self.NAVIGATING
            self.waypoint_start_time = self.get_clock().now()
            self.status_message = "Mission resumed"
            response.success = True
            response.message = "Mission resumed"
        else:
            response.success = False
            response.message = f"Cannot pause/resume in state {self.state}"

        self.get_logger().info(response.message)
        return response

    def control_loop(self):
        """Main control loop - runs at control_frequency Hz"""

        # Only navigate when in NAVIGATING state
        if self.state != self.NAVIGATING:
            return

        # Check if we have necessary data
        if self.current_pose is None or self.mission_path is None:
            return

        # Check if mission is complete
        if self.current_waypoint_idx >= len(self.mission_path.poses):
            self.state = self.REACHED_GOAL
            self.status_message = "Mission completed"
            self.get_logger().info(self.status_message)
            return

        # Get current waypoint
        current_waypoint = self.mission_path.poses[self.current_waypoint_idx]

        # Calculate distance to waypoint
        distance = self.calculate_distance(self.current_pose.pose, current_waypoint.pose)

        # Check if waypoint is reached
        if distance < self.waypoint_threshold:
            self.current_waypoint_idx += 1
            self.waypoint_start_time = self.get_clock().now()

            if self.current_waypoint_idx < len(self.mission_path.poses):
                self.get_logger().info(
                    f'Reached waypoint {self.current_waypoint_idx}/{len(self.mission_path.poses)}'
                )
            return

        # Check timeout
        elapsed = (self.get_clock().now() - self.waypoint_start_time).nanoseconds / 1e9
        if elapsed > self.waypoint_timeout:
            self.get_logger().warn(
                f'Waypoint timeout ({elapsed:.1f}s > {self.waypoint_timeout}s)'
            )
            # Skip to next waypoint or abort
            self.current_waypoint_idx += 1
            self.waypoint_start_time = self.get_clock().now()
            return

        # Publish setpoint for current waypoint
        setpoint = PoseStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = current_waypoint.header.frame_id
        setpoint.pose = current_waypoint.pose

        self.setpoint_pub.publish(setpoint)

    def calculate_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def calculate_distance_to_goal(self):
        """Calculate distance to final goal"""
        if self.mission_path is None or self.current_pose is None:
            return float('inf')

        if self.current_waypoint_idx >= len(self.mission_path.poses):
            return 0.0

        final_goal = self.mission_path.poses[-1].pose
        return self.calculate_distance(self.current_pose.pose, final_goal)

    def publish_status(self):
        """Publish navigation status"""
        status = NavigationStatus()
        status.state = self.state
        status.distance_to_goal = self.calculate_distance_to_goal()
        status.current_waypoint_index = self.current_waypoint_idx
        status.total_waypoints = len(self.mission_path.poses) if self.mission_path else 0
        status.status_message = self.status_message

        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
