#!/usr/bin/env python3
"""
Pure Pursuit Path Tracker for UAV with MAVROS2

Subscribes:
  - /local_planned_path     (nav_msgs/Path)          QoS: reliable
  - /mavros/local_position/pose (geometry_msgs/PoseStamped) QoS: best_effort

Publishes:
  - /mavros/setpoint_raw/local (mavros_msgs/PositionTarget) @ 20 Hz

Parameters (ros2 params):
  - lookahead_dist   (float, default 1.2)   Lookahead radius in meters
  - max_speed        (float, default 0.5)   Max speed m/s (straight path)
  - min_speed        (float, default 0.15)  Min speed m/s (tight curves)
  - curvature_window (int,   default 5)     Number of lookahead points for curvature calc
  - goal_threshold   (float, default 0.25)  Distance to final goal to stop [m]
  - publish_rate     (float, default 20.0)  Setpoint publish rate [Hz]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import numpy as np
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode


def quat_to_yaw(q) -> float:
    """Extract yaw from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def dist3d(a, b) -> float:
    return math.sqrt(
        (a.x - b.x) ** 2 +
        (a.y - b.y) ** 2 +
        (a.z - b.z) ** 2
    )


def dist2d(a, b) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


class PurePursuitTracker(Node):

    def __init__(self):
        super().__init__('pure_pursuit_tracker')

        # --- Parameters ---
        self.declare_parameter('lookahead_dist',   1.5)
        self.declare_parameter('max_speed',        0.4)
        self.declare_parameter('min_speed',        0.05)
        self.declare_parameter('curvature_window', 4)
        self.declare_parameter('goal_threshold',   0.15)
        self.declare_parameter('publish_rate',     20.0)
        self.declare_parameter('yaw_tolerance',    10.0)  # degrees – align yaw before moving

        self.lookahead_dist   = self.get_parameter('lookahead_dist').value
        self.max_speed        = self.get_parameter('max_speed').value
        self.min_speed        = self.get_parameter('min_speed').value
        self.curvature_window = self.get_parameter('curvature_window').value
        self.goal_threshold   = self.get_parameter('goal_threshold').value
        self.yaw_tolerance    = math.radians(self.get_parameter('yaw_tolerance').value)
        publish_rate          = self.get_parameter('publish_rate').value

        # --- Service client ---
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # --- State ---
        self.path: Path | None = None
        self.drone_pose: PoseStamped | None = None
        self.path_index: int = 0          # closest point on path (moves forward only)
        self.goal_reached: bool = False
        self.offboard_set: bool = False

        # --- QoS ---
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # --- Subscribers ---
        self.path_sub = self.create_subscription(
            Path,
            '/local_planned_path',
            self.path_callback,
            reliable_qos,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            best_effort_qos,
        )

        # --- Publisher ---
        self.setpoint_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            best_effort_qos,
        )

        # --- Timer ---
        self.timer = self.create_timer(1.0 / publish_rate, self.control_loop)

        self.get_logger().info('Pure Pursuit Tracker started.')
        self.get_logger().info(
            f'  lookahead={self.lookahead_dist}m  '
            f'max_speed={self.max_speed}m/s  '
            f'min_speed={self.min_speed}m/s'
        )

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def pose_callback(self, msg: PoseStamped):
        self.drone_pose = msg

    def path_callback(self, msg: Path):
        if len(msg.poses) == 0:
            return

        # When a new path arrives, find the closest point to the drone
        # to avoid jumping backwards on re-plans.
        if self.drone_pose is not None:
            new_index = self._find_closest_index(msg)
        else:
            new_index = 0

        self.path = msg
        self.path_index = new_index
        self.goal_reached = False

        if not self.offboard_set:
            self._set_mode('OFFBOARD')
            self.offboard_set = True

        self.get_logger().debug(
            f'New path received: {len(msg.poses)} poses, starting at index {new_index}'
        )

    # -----------------------------------------------------------------------
    # Main control loop
    # -----------------------------------------------------------------------

    def control_loop(self):
        if self.drone_pose is None or self.path is None or len(self.path.poses) == 0:
            return

        drone_pos = self.drone_pose.pose.position
        poses = self.path.poses
        goal_pos = poses[-1].pose.position

        # Check if overall goal is reached
        if dist3d(drone_pos, goal_pos) < self.goal_threshold:
            if not self.goal_reached:
                self.get_logger().info('Goal reached! Switching to POSCTL.')
                self._set_mode('POSCTL')
                self.offboard_set = False
                self.goal_reached = True
            self._publish_hold(goal_pos, quat_to_yaw(poses[-1].pose.orientation))
            return

        # Advance closest index (never go backwards → avoids re-visiting old path segments)
        self.path_index = self._advance_closest_index(poses, drone_pos)

        # Find lookahead point
        lookahead_pose, lookahead_idx = self._find_lookahead(poses, drone_pos, self.path_index)
        target_pos = lookahead_pose.pose.position

        # Compute desired yaw: face towards lookahead point (2D)
        dx = target_pos.x - drone_pos.x
        dy = target_pos.y - drone_pos.y
        if abs(dx) > 1e-4 or abs(dy) > 1e-4:
            desired_yaw = math.atan2(dy, dx)
        else:
            desired_yaw = quat_to_yaw(self.drone_pose.pose.orientation)

        # Check yaw alignment before moving
        current_yaw = quat_to_yaw(self.drone_pose.pose.orientation)
        yaw_error = abs(math.atan2(math.sin(desired_yaw - current_yaw),
                                   math.cos(desired_yaw - current_yaw)))

        if yaw_error > self.yaw_tolerance:
            # Not aligned yet → hold position, only rotate
            self._publish_hold(drone_pos, desired_yaw)
            self.get_logger().debug(
                f'Aligning yaw: error={math.degrees(yaw_error):.1f}° > '
                f'tolerance={math.degrees(self.yaw_tolerance):.1f}°'
            )
            return

        # Compute speed based on path curvature ahead
        speed = self._compute_speed(poses, lookahead_idx)

        # Direction vector drone → lookahead (3D, normalized)
        vec = np.array([
            target_pos.x - drone_pos.x,
            target_pos.y - drone_pos.y,
            target_pos.z - drone_pos.z,
        ])
        norm = np.linalg.norm(vec)
        if norm > 1e-4:
            vec = vec / norm * speed
        else:
            vec = np.zeros(3)

        self._publish_setpoint(target_pos, vec, desired_yaw)

    # -----------------------------------------------------------------------
    # Pure Pursuit helpers
    # -----------------------------------------------------------------------

    def _find_closest_index(self, path: Path) -> int:
        """Find index of path point closest to drone (for re-plan handoff)."""
        if self.drone_pose is None:
            return 0
        drone_pos = self.drone_pose.pose.position
        min_d = float('inf')
        best_i = 0
        for i, pose in enumerate(path.poses):
            d = dist3d(drone_pos, pose.pose.position)
            if d < min_d:
                min_d = d
                best_i = i
        return best_i

    def _advance_closest_index(self, poses, drone_pos) -> int:
        """
        Advance path_index forward as long as the next point is closer.
        This prevents the tracker from jumping backwards.
        """
        idx = self.path_index
        while idx < len(poses) - 1:
            d_current = dist3d(drone_pos, poses[idx].pose.position)
            d_next    = dist3d(drone_pos, poses[idx + 1].pose.position)
            if d_next < d_current:
                idx += 1
            else:
                break
        return idx

    def _find_lookahead(self, poses, drone_pos, start_idx: int):
        """
        Walk forward from start_idx until a point is beyond lookahead_dist.
        Returns (pose, index) of the lookahead target.
        """
        for i in range(start_idx, len(poses)):
            d = dist3d(drone_pos, poses[i].pose.position)
            if d >= self.lookahead_dist:
                return poses[i], i

        # If no point is far enough → use last point
        return poses[-1], len(poses) - 1

    # -----------------------------------------------------------------------
    # Speed scaling via curvature
    # -----------------------------------------------------------------------

    def _compute_speed(self, poses, lookahead_idx: int) -> float:
        """
        Estimate path curvature over the next `curvature_window` points
        ahead of lookahead_idx and scale speed between min_speed and max_speed.

        Curvature metric: mean angle change between consecutive segments.
        0 rad  → straight → max_speed
        π/2 rad → 90° turn → min_speed
        """
        end_idx = min(lookahead_idx + self.curvature_window, len(poses) - 1)
        window = poses[lookahead_idx:end_idx + 1]

        if len(window) < 3:
            return self.max_speed

        angle_sum = 0.0
        count = 0

        for i in range(1, len(window) - 1):
            p0 = window[i - 1].pose.position
            p1 = window[i].pose.position
            p2 = window[i + 1].pose.position

            v1 = np.array([p1.x - p0.x, p1.y - p0.y, p1.z - p0.z])
            v2 = np.array([p2.x - p1.x, p2.y - p1.y, p2.z - p1.z])

            n1 = np.linalg.norm(v1)
            n2 = np.linalg.norm(v2)
            if n1 < 1e-6 or n2 < 1e-6:
                continue

            cos_a = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
            angle = math.acos(cos_a)   # 0 = straight, π = U-turn
            angle_sum += angle
            count += 1

        if count == 0:
            return self.max_speed

        mean_angle = angle_sum / count  # radians

        # Normalize: 0 → max_speed, π/2 (90°) or more → min_speed
        t = min(mean_angle / (math.pi / 2), 1.0)
        speed = (self.max_speed * (1.0 - t) + self.min_speed * t) * 0.7

        self.get_logger().debug(
            f'mean_angle={math.degrees(mean_angle):.1f}°  speed={speed:.2f} m/s'
        )
        return speed 

    # -----------------------------------------------------------------------
    # Flight mode
    # -----------------------------------------------------------------------

    def _set_mode(self, mode: str):
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)
        self.get_logger().info(f'Requested flight mode: {mode}')

    # -----------------------------------------------------------------------
    # MAVROS publishers
    # -----------------------------------------------------------------------

    def _publish_setpoint(self, target_pos, velocity_vec, yaw: float):
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Ignore acceleration and yaw_rate; use position + velocity feedforward
        msg.type_mask = (
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        msg.position.x = target_pos.x
        msg.position.y = target_pos.y
        msg.position.z = target_pos.z

        msg.velocity.x = float(velocity_vec[0])
        msg.velocity.y = float(velocity_vec[1])
        msg.velocity.z = float(velocity_vec[2])

        msg.yaw = yaw


        self.setpoint_pub.publish(msg)

    def _publish_hold(self, pos, yaw: float):
        """Hold current goal position with zero velocity."""
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        msg.type_mask = (
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        msg.position.x = pos.x
        msg.position.y = pos.y
        msg.position.z = pos.z

        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0

        msg.yaw = yaw

        self.setpoint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()