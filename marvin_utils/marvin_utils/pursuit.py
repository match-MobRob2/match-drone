#!/usr/bin/env python3
"""
Pure Pursuit Path Tracker for UAV with MAVROS2

Regelung: Position-Setpoints. Der Carrot wird kontinuierlich bei ca.
lookahead_dist (~1 m) vor der Drohne entlang des Pfads interpoliert und als
Positionsziel kommandiert — PX4s Positionsregler macht daraus die
Geschwindigkeit (Fehler <= 1 m -> sanfte Fahrt, natuerliches Abbremsen am
Ziel, wenn der Carrot auf dem letzten Pfadpunkt einrastet). Yaw zeigt
tendenziell entlang des Pfads (Richtung Carrot). Position-Hold am Ziel und
bei leerem Pfad.

Frames: Der Pfad kommt im map-Frame und wird auch dort getrackt. Als
Drohnenpose dient DIESELBE Quelle wie im Planner (TF map->base_link, Fallback
MAVROS-Pose) — sonst vergleicht der Tracker zwei unabhaengige Schaetzungen
derselben Position und jede Odometrie-Drift (FAST-LIO) wird zu Regelfehler.
PX4 fliegt aber in seinem eigenen EKF-Frame: Der Carrot wird im letzten
Schritt mit der momentanen Yaw-Differenz zwischen beiden Posen in den
PX4-Frame gedreht und um die Drohne verschoben. Ist die TF stale
(> tf_max_age), faellt der Tracker konsistent zum Planner auf die MAVROS-Pose
zurueck (Offset 0).

Subscribes:
  - /local_planned_path         (nav_msgs/Path)             QoS: reliable
  - /mavros/local_position/pose (geometry_msgs/PoseStamped) QoS: best_effort

Publishes:
  - /mavros/setpoint_raw/local  (mavros_msgs/PositionTarget) @ 20 Hz
      immer: position + yaw

Parameters:
  - lookahead_dist   (float, 1.0)   Carrot-Abstand vor der Drohne [m]
  - goal_threshold   (float, 0.15)  Distanz zum Ziel zum Stoppen [m]
  - publish_rate     (float, 20.0)  Setpoint-Rate [Hz]
  - tf_max_age       (float, 1.0)   Max. Alter der TF map->base_link [s]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import numpy as np
import math

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode

import tf2_ros


def quat_to_yaw(q) -> float:
    """Extract yaw from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def pt(p) -> np.ndarray:
    """geometry_msgs/Point -> np.ndarray(3)."""
    return np.array([p.x, p.y, p.z])


def rot_z(v, yaw: float) -> np.ndarray:
    """Vektor um yaw um die z-Achse drehen."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])


# --- Pure Pursuit Geometrie (ROS-frei, damit testbar) --------------------------

def carrot_on_path(pts, drone, lookahead: float):
    """
    Kontinuierlicher Carrot: Schnittpunkt der Kugel (Radius `lookahead` um die
    Drohne) mit dem Polygonzug `pts`, am weitesten vorne. So wandert das Ziel
    stetig mit, statt von Wegpunkt zu Wegpunkt zu springen.

    pts:      Liste np.ndarray(3), ab dem naechstgelegenen Wegpunkt aufwaerts.
    drone:    np.ndarray(3), aktuelle Drohnenposition.
    Rueckgabe: np.ndarray(3). Liegt der ganze Restpfad innerhalb der Kugel
              (nahe am Ziel), wird der letzte Punkt zurueckgegeben; ist die
              Drohne weiter als `lookahead` vom Pfad abgekommen, der
              naechstgelegene Punkt (zurueck auf den Pfad — NICHT das Pfadende,
              ein Beeline dorthin wuerde jede Kurve um Hindernisse ignorieren).
    """
    L2 = lookahead * lookahead
    for i in range(len(pts) - 1):
        a = pts[i]
        ab = pts[i + 1] - a
        f = a - drone
        A = float(ab.dot(ab))
        if A < 1e-9:
            continue
        B = 2.0 * float(f.dot(ab))
        C = float(f.dot(f)) - L2
        disc = B * B - 4.0 * A * C
        if disc < 0.0:
            continue  # Segment schneidet die Kugel nicht
        t = (-B + math.sqrt(disc)) / (2.0 * A)  # groessere Wurzel = weiter vorne
        if 0.0 <= t <= 1.0:
            return a + t * ab
    if float(np.linalg.norm(pts[-1] - drone)) <= lookahead:
        return pts[-1]
    return min(pts, key=lambda p: float(np.linalg.norm(p - drone)))


class PurePursuitTracker(Node):

    def __init__(self):
        super().__init__('pure_pursuit_tracker')

        # --- Parameters ---
        self.declare_parameter('lookahead_dist', 1.0)
        self.declare_parameter('goal_threshold', 0.15)
        self.declare_parameter('publish_rate',   20.0)
        self.declare_parameter('tf_max_age',     1.0)

        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        self.goal_threshold = self.get_parameter('goal_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.tf_max_age = self.get_parameter('tf_max_age').value

        # TF map->base_link = Pose aus Sicht des Planners (nav_node nutzt
        # dieselbe Quelle mit demselben Fallback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Service client (einmaliges OFFBOARD-Setzen) ---
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # --- State ---
        self.path: Path | None = None
        self.drone_pose: PoseStamped | None = None
        self.path_index: int = 0          # closest point on path (moves forward only)
        self.goal_reached: bool = False
        self.offboard_set: bool = False
        self.hold_pos = None              # Haltepunkt im PX4-Frame (Ziel / Stopp-Signal)
        self.hold_yaw = 0.0

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
            Path, '/local_planned_path', self.path_callback, reliable_qos)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.pose_callback, best_effort_qos)

        # --- Publisher ---
        self.setpoint_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', best_effort_qos)

        # --- Timer ---
        self.timer = self.create_timer(1.0 / self.publish_rate, self.control_loop)

        self.get_logger().info(
            f'Pure Pursuit (position) gestartet. lookahead={self.lookahead_dist}m')

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def pose_callback(self, msg: PoseStamped):
        self.drone_pose = msg

    def path_callback(self, msg: Path):
        if len(msg.poses) == 0:
            # Leerer Pfad = Stopp-Signal vom Planer: aktuelle Position halten
            if self.path is not None and self.drone_pose is not None:
                self.get_logger().warn('Leerer Pfad empfangen — halte Position')
                self.hold_pos = self.drone_pose.pose.position
                self.hold_yaw = quat_to_yaw(self.drone_pose.pose.orientation)
            self.path = None
            return

        # Bei neuem Pfad naechstgelegenen Punkt suchen, damit wir nicht
        # rueckwaerts auf alte Pfadsegmente springen.
        new_index = 0
        if self.drone_pose is not None:
            pos_map, _, _ = self._pose_map()
            new_index = self._find_closest_index(msg, pos_map)

        self.path = msg
        self.path_index = new_index
        self.goal_reached = False
        self.hold_pos = None

        if not self.offboard_set:
            self._set_mode('OFFBOARD')
            self.offboard_set = True

    # -----------------------------------------------------------------------
    # Pose / Frames
    # -----------------------------------------------------------------------

    def _pose_map(self):
        """
        Drohnenpose im map-Frame + Yaw-Offset zum PX4-Frame.

        Rueckgabe (pos_map, yaw_map, dyaw): dyaw = Yaw(PX4) - Yaw(map) dreht
        map-Vektoren in den PX4-Frame. Quelle ist die TF map->base_link
        (Planner-Sicht, enthaelt Relokalisierung + FAST-LIO); ist sie stale
        oder fehlt sie, MAVROS-Pose mit dyaw=0 — dann faellt auch der Planner
        auf dieselbe Pose zurueck und beide bleiben konsistent.
        """
        mav = self.drone_pose.pose
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            age = (self.get_clock().now()
                   - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
            if age <= self.tf_max_age:
                t = tf.transform.translation
                yaw_map = quat_to_yaw(tf.transform.rotation)
                dyaw = quat_to_yaw(mav.orientation) - yaw_map
                return np.array([t.x, t.y, t.z]), yaw_map, dyaw
        except Exception:  # noqa: B902 — tf2 wirft diverse Exception-Typen
            pass
        return pt(mav.position), quat_to_yaw(mav.orientation), 0.0

    def _map_to_px4(self, x_map, pos_map, dyaw: float) -> np.ndarray:
        """Punkt aus map in den PX4-Frame: um die Drohne drehen + verschieben."""
        return pt(self.drone_pose.pose.position) + rot_z(x_map - pos_map, dyaw)

    # -----------------------------------------------------------------------
    # Main control loop
    # -----------------------------------------------------------------------

    def control_loop(self):
        if self.drone_pose is None:
            return

        if self.path is None or len(self.path.poses) == 0:
            if self.hold_pos is not None:
                self._publish_position(self.hold_pos, self.hold_yaw)
            return

        if self.goal_reached:
            self._publish_position(self.hold_pos, self.hold_yaw)
            return

        pos_map, yaw_map, dyaw = self._pose_map()
        poses = self.path.poses
        goal = pt(poses[-1].pose.position)
        dist_goal = float(np.linalg.norm(goal - pos_map))

        # Ziel erreicht → Zielpunkt einmalig in den PX4-Frame umrechnen und in
        # OFFBOARD dort halten (kein Modus-Flip, kein Nachlaufen bei Drift)
        if dist_goal < self.goal_threshold:
            self.get_logger().info('Ziel erreicht — halte Position')
            self.goal_reached = True
            hp = self._map_to_px4(goal, pos_map, dyaw)
            self.hold_pos = Point(x=float(hp[0]), y=float(hp[1]), z=float(hp[2]))
            self.hold_yaw = quat_to_yaw(poses[-1].pose.orientation) + dyaw
            self._publish_position(self.hold_pos, self.hold_yaw)
            return

        # Naechstgelegenen Index vorruecken (nie rueckwaerts)
        self.path_index = self._advance_closest_index(poses, pos_map)
        pts_path = [pt(p.pose.position) for p in poses[self.path_index:]]

        carrot = carrot_on_path(pts_path, pos_map, self.lookahead_dist)
        vec = carrot - pos_map
        norm = float(np.linalg.norm(vec))

        # Weit vom Pfad abgekommen liegt der Carrot (naechster Pfadpunkt)
        # > lookahead entfernt — Setpoint auf lookahead deckeln, sonst
        # kommandiert der Positionsregler einen grossen Sprung mit voller
        # PX4-Geschwindigkeit.
        if norm > self.lookahead_dist:
            carrot = pos_map + vec * (self.lookahead_dist / norm)

        # Yaw tendenziell entlang des Pfads (Richtung Carrot, 2D). Bei ganz
        # kurzem Vektor aktuellen Yaw halten (sonst atan2-Rauschen am Ziel).
        if math.hypot(vec[0], vec[1]) > 0.3:
            yaw = math.atan2(vec[1], vec[0]) + dyaw
        else:
            yaw = quat_to_yaw(self.drone_pose.pose.orientation)

        sp = self._map_to_px4(carrot, pos_map, dyaw)
        self._publish_position(
            Point(x=float(sp[0]), y=float(sp[1]), z=float(sp[2])), yaw)

    # -----------------------------------------------------------------------
    # Pure Pursuit helpers
    # -----------------------------------------------------------------------

    def _find_closest_index(self, path: Path, pos_map) -> int:
        """Index des zur Drohne naechstgelegenen Pfadpunkts (fuer Replan-Uebergabe)."""
        return min(range(len(path.poses)),
                   key=lambda i: float(np.linalg.norm(
                       pt(path.poses[i].pose.position) - pos_map)))

    def _advance_closest_index(self, poses, pos_map) -> int:
        """path_index vorruecken solange der naechste Punkt naeher ist (nie rueckwaerts)."""
        idx = self.path_index
        while idx < len(poses) - 1:
            d_current = float(np.linalg.norm(pt(poses[idx].pose.position) - pos_map))
            d_next = float(np.linalg.norm(pt(poses[idx + 1].pose.position) - pos_map))
            if d_next < d_current:
                idx += 1
            else:
                break
        return idx

    # -----------------------------------------------------------------------
    # Flight mode
    # -----------------------------------------------------------------------

    def _set_mode(self, mode: str):
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)
        self.get_logger().info(f'Requested flight mode: {mode}')

    # -----------------------------------------------------------------------
    # MAVROS publisher
    # -----------------------------------------------------------------------

    def _publish_position(self, pos, yaw: float):
        """Position + Yaw (Velocity/Accel ignoriert) — Fahrt UND Halten."""
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        msg.position.x = pos.x
        msg.position.y = pos.y
        msg.position.z = pos.z
        msg.yaw = float(yaw)
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
