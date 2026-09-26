"""Pure-Pursuit-Geometrie: Carrot-Interpolation und Fallbacks."""

import math

import numpy as np

from marvin_utils.pursuit import carrot_on_path, rot_z


def test_carrot_interpolates_on_segment():
    # Gerade 0->2m, Lookahead 1m → Carrot genau bei 1m (nicht auf Wegpunkt gesnappt)
    pts = [np.array([0.0, 0.0, 0.0]), np.array([2.0, 0.0, 0.0])]
    c = carrot_on_path(pts, np.zeros(3), 1.0)
    assert np.allclose(c, [1.0, 0.0, 0.0], atol=1e-6)


def test_carrot_falls_back_to_last_point_near_goal():
    # Ganzer Restpfad innerhalb der Lookahead-Kugel → letzter Punkt (Ziel)
    pts = [np.array([0.0, 0.0, 0.0]), np.array([0.3, 0.0, 0.0])]
    c = carrot_on_path(pts, np.zeros(3), 1.0)
    assert np.allclose(c, [0.3, 0.0, 0.0])


def test_carrot_far_off_path_returns_closest_point_not_end():
    # Drohne > lookahead vom Pfad abgekommen → zurueck zum naechsten Punkt.
    # Frueher: Pfadende → Beeline quer durch die Szene (Wand-Crash).
    pts = [np.array([0.0, 5.0, 0.0]), np.array([5.0, 5.0, 0.0]),
           np.array([10.0, 5.0, 0.0])]
    c = carrot_on_path(pts, np.zeros(3), 1.5)
    assert np.allclose(c, [0.0, 5.0, 0.0])


def test_offpath_setpoint_capped_at_lookahead():
    # Off-Path-Fallback liefert einen Punkt > lookahead entfernt; der
    # Setpoint wird im Node auf lookahead gedeckelt (gleiche Formel hier).
    pts = [np.array([0.0, 5.0, 0.0]), np.array([5.0, 5.0, 0.0])]
    drone = np.zeros(3)
    lookahead = 1.0
    carrot = carrot_on_path(pts, drone, lookahead)
    vec = carrot - drone
    norm = float(np.linalg.norm(vec))
    assert norm > lookahead
    capped = drone + vec * (lookahead / norm)
    assert abs(np.linalg.norm(capped - drone) - lookahead) < 1e-9


def test_rot_z_quarter_turn():
    v = rot_z(np.array([1.0, 0.0, 2.0]), math.pi / 2)
    assert np.allclose(v, [0.0, 1.0, 2.0], atol=1e-9)


def test_failsafe_hold_survives_new_path():
    # Regression 2026-09-25: Halt aktiv (TF veraltet), dann neuer Planer-Pfad -> hold_pos
    # wurde None, pursuit stuerzte ab, PX4 verlor Offboard-Setpoints.
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from marvin_utils.pursuit import PurePursuitTracker

    def path(x):
        p = Path()
        for i in range(3):
            ps = PoseStamped()
            ps.pose.position.x = x + i
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        return p

    rclpy.init()
    try:
        n = PurePursuitTracker()
        sent = []
        n._publish_position = lambda pos, yaw: sent.append((pos.x, pos.y, pos.z))
        n.drone_pose = PoseStamped()
        n.drone_pose.pose.position.x, n.drone_pose.pose.position.z = 1.0, 1.5
        n.drone_pose.pose.orientation.w = 1.0
        n.path_callback(path(0.0))       # keine TF im Test -> Pose veraltet -> Halt
        n.control_loop()
        n.path_callback(path(5.0))       # neuer Pfad waehrend des Halts
        n.drone_pose.pose.position.x = 3.0  # Drohne driftet, Haltepunkt darf nicht mitwandern
        n.control_loop()
        assert n.failsafe_hold and sent == [(1.0, 0.0, 1.5), (1.0, 0.0, 1.5)]
        n.destroy_node()
    finally:
        rclpy.shutdown()
