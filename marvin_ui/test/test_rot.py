"""_rot: Quaternion -> Matrix (transformiert Live-Lidar nach map, falsches Vorzeichen = gespiegelte Hindernisse)."""
import math

import numpy as np

from marvin_ui.supervisor import _rot


def test_rot_yaw_90():
    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert np.allclose(_rot(q) @ [1, 0, 0], [0, 1, 0])


def test_rot_orthonormal():
    q = np.array([0.1, -0.3, 0.5, 0.8])
    R = _rot(q / np.linalg.norm(q))
    assert np.allclose(R @ R.T, np.eye(3)) and np.isclose(np.linalg.det(R), 1)
