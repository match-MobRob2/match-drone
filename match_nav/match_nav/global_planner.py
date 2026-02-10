#!/usr/bin/env python3
"""
Global Path Planner Node for Match Drone Navigation Stack

Provides 3D path planning service using OMPL (or simplified planning for MVP).
Plans collision-free paths from start to goal using Octomap.

Author: Match Drone Team
License: MIT
"""

import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from octomap_msgs.msg import Octomap
from match_nav_msgs.srv import PlanPath


class GlobalPlanner(Node):
    """
    Global Path Planner Node

    Provides PlanPath service for 3D path planning.

    MVP Implementation: Simple waypoint interpolation (straight line)
    Phase 2: OMPL RRT* integration with Octomap collision checking
    """

    def __init__(self):
        super().__init__('global_planner')

        # Declare parameters
        self.declare_parameter('planner_type', 'RRTstar')
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('state_space', 'SE3')
        self.declare_parameter('bounds.min_x', -100.0)
        self.declare_parameter('bounds.max_x', 100.0)
        self.declare_parameter('bounds.min_y', -100.0)
        self.declare_parameter('bounds.max_y', 100.0)
        self.declare_parameter('bounds.min_z', 0.5)
        self.declare_parameter('bounds.max_z', 20.0)
        self.declare_parameter('waypoint_spacing', 1.0)  # meters
        self.declare_parameter('simplify_path', True)

        # Get parameters
        self.planner_type = self.get_parameter('planner_type').value
        self.planning_time = self.get_parameter('planning_time').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.simplify_path = self.get_parameter('simplify_path').value

        # Bounds
        self.min_x = self.get_parameter('bounds.min_x').value
        self.max_x = self.get_parameter('bounds.max_x').value
        self.min_y = self.get_parameter('bounds.min_y').value
        self.max_y = self.get_parameter('bounds.max_y').value
        self.min_z = self.get_parameter('bounds.min_z').value
        self.max_z = self.get_parameter('bounds.max_z').value

        # State variables
        self.octomap = None
        self.octomap_received = False

        # Subscribers
        self.octomap_sub = self.create_subscription(
            Octomap,
            '/octomap_binary',
            self.octomap_callback,
            10
        )

        # Services
        self.plan_service = self.create_service(
            PlanPath,
            '/match_nav/plan_path',
            self.plan_path_callback
        )

        self.get_logger().info(
            f'Global Planner initialized (type: {self.planner_type}, '
            f'MVP mode: Simple Waypoint Interpolation)'
        )
        self.get_logger().warn(
            'OMPL integration pending - using simplified planning for MVP'
        )

    def octomap_callback(self, msg):
        """Receive and store Octomap"""
        self.octomap = msg
        if not self.octomap_received:
            self.octomap_received = True
            self.get_logger().info('Octomap received')

    def plan_path_callback(self, request, response):
        """
        Plan path service callback

        MVP Implementation: Simple straight-line interpolation
        TODO: Replace with OMPL RRT* in Phase 2
        """
        start_time = self.get_clock().now()

        # Validate bounds
        if not self.check_bounds(request.start):
            response.success = False
            response.error_message = "Start pose out of bounds"
            self.get_logger().warn(response.error_message)
            return response

        if not self.check_bounds(request.goal):
            response.success = False
            response.error_message = "Goal pose out of bounds"
            self.get_logger().warn(response.error_message)
            return response

        # Plan path (simplified for MVP)
        try:
            path = self.plan_straight_line(request.start, request.goal)

            # Optional: Simplify path
            if self.simplify_path and len(path.poses) > 2:
                path = self.simplify_path_simple(path)

            response.path = path
            response.success = True
            response.error_message = ""

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            self.get_logger().info(
                f'Path planned: {len(path.poses)} waypoints in {elapsed:.3f}s'
            )

        except Exception as e:
            response.success = False
            response.error_message = f"Planning failed: {str(e)}"
            self.get_logger().error(response.error_message)

        return response

    def plan_straight_line(self, start_pose, goal_pose):
        """
        Simple straight-line path with waypoint interpolation

        Args:
            start_pose: Start pose
            goal_pose: Goal pose

        Returns:
            Path message with interpolated waypoints
        """
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"

        # Calculate distance
        dx = goal_pose.position.x - start_pose.position.x
        dy = goal_pose.position.y - start_pose.position.y
        dz = goal_pose.position.z - start_pose.position.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Calculate number of waypoints
        num_waypoints = max(2, int(distance / self.waypoint_spacing) + 1)

        # Interpolate waypoints
        for i in range(num_waypoints):
            t = i / (num_waypoints - 1)  # 0.0 to 1.0

            waypoint = PoseStamped()
            waypoint.header = path.header

            # Linear interpolation
            waypoint.pose.position.x = start_pose.position.x + t * dx
            waypoint.pose.position.y = start_pose.position.y + t * dy
            waypoint.pose.position.z = start_pose.position.z + t * dz

            # Orientation (simple SLERP approximation - use start orientation for MVP)
            waypoint.pose.orientation = start_pose.orientation if t < 0.5 else goal_pose.orientation

            path.poses.append(waypoint)

        return path

    def simplify_path_simple(self, path):
        """
        Simple path simplification - remove intermediate waypoints in straight lines

        Args:
            path: Original path

        Returns:
            Simplified path
        """
        if len(path.poses) < 3:
            return path

        simplified = Path()
        simplified.header = path.header
        simplified.poses.append(path.poses[0])  # Always keep start

        for i in range(1, len(path.poses) - 1):
            # Check if waypoint is necessary (angle check)
            prev = path.poses[i-1].pose.position
            curr = path.poses[i].pose.position
            next_pos = path.poses[i+1].pose.position

            # Vectors
            v1 = np.array([curr.x - prev.x, curr.y - prev.y, curr.z - prev.z])
            v2 = np.array([next_pos.x - curr.x, next_pos.y - curr.y, next_pos.z - curr.z])

            # Normalize
            v1_norm = v1 / (np.linalg.norm(v1) + 1e-6)
            v2_norm = v2 / (np.linalg.norm(v2) + 1e-6)

            # Angle between vectors
            dot_product = np.dot(v1_norm, v2_norm)
            angle = math.acos(np.clip(dot_product, -1.0, 1.0))

            # Keep waypoint if angle is significant (> 10 degrees)
            if angle > 0.174:  # 10 degrees in radians
                simplified.poses.append(path.poses[i])

        simplified.poses.append(path.poses[-1])  # Always keep goal

        return simplified

    def check_bounds(self, pose):
        """Check if pose is within workspace bounds"""
        if pose.position.x < self.min_x or pose.position.x > self.max_x:
            return False
        if pose.position.y < self.min_y or pose.position.y > self.max_y:
            return False
        if pose.position.z < self.min_z or pose.position.z > self.max_z:
            return False
        return True

    # TODO: OMPL Integration for Phase 2
    def plan_with_ompl(self, start_pose, goal_pose):
        """
        OMPL RRT* Planning (Phase 2)

        This will be implemented in Phase 2 with proper:
        - OMPL Python bindings
        - Octomap collision checking
        - State space configuration
        - RRT* algorithm setup
        """
        raise NotImplementedError("OMPL planning not yet implemented - use MVP mode")


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
