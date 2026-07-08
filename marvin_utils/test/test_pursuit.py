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
