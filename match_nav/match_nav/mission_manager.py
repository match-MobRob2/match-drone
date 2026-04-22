#!/usr/bin/env python3
"""
Mission Manager Node for Match Drone Navigation Stack

High-level state machine that orchestrates navigation missions.
Coordinates global planning and waypoint following.

Author: Match Drone Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
from match_nav_msgs.srv import NavigateToPose, PlanPath
from match_nav_msgs.msg import NavigationStatus


class MissionManager(Node):
    """
    Mission Manager Node

    High-Level State Machine:
    - IDLE: Waiting for mission
    - PLANNING: Planning path
    - NAVIGATING: Executing navigation
    - COMPLETED: Mission completed
    - ERROR: Error state
    """

    # State constants
    IDLE = 0
    PLANNING = 1
    NAVIGATING = 2
    COMPLETED = 3
    ERROR = 4

    def __init__(self):
        super().__init__('mission_manager')

        # Declare parameters
        self.declare_parameter('replan_on_failure', True)
        self.declare_parameter('max_replan_attempts', 3)
        self.declare_parameter('replan_timeout', 10.0)
        self.declare_parameter('planning_timeout', 5.0)
        self.declare_parameter('execution_timeout', 120.0)

        # Get parameters
        self.replan_on_failure = self.get_parameter('replan_on_failure').value
        self.max_replan_attempts = self.get_parameter('max_replan_attempts').value
        self.replan_timeout = self.get_parameter('replan_timeout').value
        self.planning_timeout = self.get_parameter('planning_timeout').value
        self.execution_timeout = self.get_parameter('execution_timeout').value

        # State variables
        self.state = self.IDLE
        self.current_pose = None
        self.current_goal = None
        self.replan_attempts = 0
        self.mission_start_time = None
        self.follower_status = None

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10
        )

        self.status_sub = self.create_subscription(
            NavigationStatus,
            '/match_nav/status',
            self.status_callback,
            10
        )

        # Publishers
        self.mission_path_pub = self.create_publisher(
            Path,
            '/match_nav/mission_path',
            10
        )

        # Service clients (to other nav nodes)
        self.plan_path_client = self.create_client(
            PlanPath,
            '/match_nav/plan_path'
        )

        self.start_mission_client = self.create_client(
            Trigger,
            '/match_nav/start_mission'
        )

        self.abort_mission_client = self.create_client(
            Trigger,
            '/match_nav/abort_mission'
        )

        # Services provided (to user/external nodes)
        self.navigate_to_pose_srv = self.create_service(
            NavigateToPose,
            '/match_nav/navigate_to_pose',
            self.navigate_to_pose_callback
        )

        self.cancel_mission_srv = self.create_service(
            Trigger,
            '/match_nav/cancel_mission',
            self.cancel_mission_callback
        )

        # Timer for monitoring
        self.monitor_timer = self.create_timer(1.0, self.monitor_execution)

        self.get_logger().info('Mission Manager initialized')

        # Wait for services
        self.get_logger().info('Waiting for global_planner service...')
        self.plan_path_client.wait_for_service(timeout_sec=5.0)

        self.get_logger().info('Waiting for waypoint_follower service...')
        self.start_mission_client.wait_for_service(timeout_sec=5.0)

        self.get_logger().info('All services available - Mission Manager ready')

    def pose_callback(self, msg):
        """Update current pose"""
        self.current_pose = msg

    def status_callback(self, msg):
        """Update waypoint follower status"""
        self.follower_status = msg

        # React to follower state changes
        if self.state == self.NAVIGATING:
            if msg.state == NavigationStatus.REACHED_GOAL:
                self.state = self.COMPLETED
                self.get_logger().info('Mission completed successfully!')

            elif msg.state == NavigationStatus.ERROR:
                self.state = self.ERROR
                self.get_logger().error(f'Navigation error: {msg.status_message}')

                # Attempt replan if enabled
                if self.replan_on_failure and self.replan_attempts < self.max_replan_attempts:
                    self.get_logger().info(f'Attempting replan ({self.replan_attempts + 1}/{self.max_replan_attempts})...')
                    self.replan_attempts += 1
                    time.sleep(self.replan_timeout)
                    self.execute_navigation(self.current_goal)

    def navigate_to_pose_callback(self, request, response):
        """
        Navigate to Pose service callback

        Main entry point for navigation missions
        """
        if self.state == self.NAVIGATING:
            response.success = False
            response.message = "Mission already in progress"
            self.get_logger().warn(response.message)
            return response

        if self.current_pose is None:
            response.success = False
            response.message = "No pose available"
            self.get_logger().error(response.message)
            return response

        # Store goal
        self.current_goal = request.goal

        # Execute navigation
        success = self.execute_navigation(request.goal)

        if success:
            response.success = True
            response.message = "Navigation started"
        else:
            response.success = False
            response.message = "Navigation failed to start"

        return response

    def execute_navigation(self, goal):
        """
        Execute navigation workflow:
        1. IDLE → PLANNING
        2. Plan path with global_planner
        3. PLANNING → NAVIGATING
        4. Start waypoint_follower
        5. Monitor execution
        """
        self.get_logger().info(f'Starting navigation to [{goal.pose.position.x:.2f}, '
                                f'{goal.pose.position.y:.2f}, {goal.pose.position.z:.2f}]')

        # Transition to PLANNING
        self.state = self.PLANNING
        self.mission_start_time = self.get_clock().now()

        # Step 1: Plan path
        plan_request = PlanPath.Request()
        plan_request.start = self.current_pose.pose
        plan_request.goal = goal.pose
        plan_request.planning_time = self.planning_timeout

        future = self.plan_path_client.call_async(plan_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.planning_timeout + 1.0)

        if not future.done():
            self.get_logger().error('Planning service timeout')
            self.state = self.ERROR
            return False

        plan_response = future.result()

        if not plan_response.success:
            self.get_logger().error(f'Planning failed: {plan_response.error_message}')
            self.state = self.ERROR
            return False

        self.get_logger().info(f'Path planned with {len(plan_response.path.poses)} waypoints')

        # Step 2: Publish mission path
        self.mission_path_pub.publish(plan_response.path)

        # Step 3: Start waypoint follower
        start_request = Trigger.Request()
        future = self.start_mission_client.call_async(start_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done():
            self.get_logger().error('Start mission service timeout')
            self.state = self.ERROR
            return False

        start_response = future.result()

        if not start_response.success:
            self.get_logger().error(f'Failed to start mission: {start_response.message}')
            self.state = self.ERROR
            return False

        # Transition to NAVIGATING
        self.state = self.NAVIGATING
        self.get_logger().info('Navigation started - monitoring execution')

        return True

    def cancel_mission_callback(self, request, response):
        """Cancel current mission"""
        if self.state not in [self.NAVIGATING, self.PLANNING]:
            response.success = False
            response.message = f"No active mission (state: {self.state})"
            return response

        # Call abort on waypoint follower
        abort_request = Trigger.Request()
        future = self.abort_mission_client.call_async(abort_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        self.state = self.IDLE
        self.current_goal = None
        self.replan_attempts = 0

        response.success = True
        response.message = "Mission cancelled"
        self.get_logger().info(response.message)

        return response

    def monitor_execution(self):
        """Monitor mission execution and check timeouts"""
        if self.state != self.NAVIGATING:
            return

        if self.mission_start_time is None:
            return

        # Check execution timeout
        elapsed = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
        if elapsed > self.execution_timeout:
            self.get_logger().error(f'Mission timeout ({elapsed:.1f}s > {self.execution_timeout}s)')
            self.state = self.ERROR

            # Abort mission
            abort_request = Trigger.Request()
            self.abort_mission_client.call_async(abort_request)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
