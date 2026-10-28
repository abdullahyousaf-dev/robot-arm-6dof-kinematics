"""Quintic trajectories: smooth start and stop with zero velocity and acceleration."""
from __future__ import annotations

import numpy as np

from .kinematics import PenArm, fk_pen, ik_pen


def quintic_coefficients(q0, qf, T, v0=0.0, vf=0.0, a0=0.0, af=0.0) -> np.ndarray:
    """Coefficients c0..c5 of q(t) = sum c_k t^k meeting position, velocity and
    acceleration at t=0 and t=T."""
    A = np.array(
        [
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [1, T, T**2, T**3, T**4, T**5],
            [0, 1, 2 * T, 3 * T**2, 4 * T**3, 5 * T**4],
            [0, 0, 2, 6 * T, 12 * T**2, 20 * T**3],
        ],
        dtype=float,
    )
    return np.linalg.solve(A, np.array([q0, v0, a0, qf, vf, af], dtype=float))


def quintic_profile(n: int) -> np.ndarray:
    """Normalised time scaling s(t) in [0, 1] with zero end velocity/acceleration."""
    t = np.linspace(0.0, 1.0, n)
    return 10 * t**3 - 15 * t**4 + 6 * t**5


def joint_space_segment(p0, p1, n=100, arm: PenArm = PenArm()) -> np.ndarray:
    """What the first firmware did: solve IK at both ends, then interpolate the
    joint angles. Smooth for the servos, but the pen does not move in a straight line."""
    q0, q1 = ik_pen(p0, arm), ik_pen(p1, arm)
    s = quintic_profile(n)[:, None]
    return q0 + s * (q1 - q0)


def cartesian_segment(p0, p1, n=100, arm: PenArm = PenArm()) -> np.ndarray:
    """Straight pen stroke: interpolate in Cartesian space with quintic timing,
    then solve IK at every sample."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    s = quintic_profile(n)[:, None]
    return np.array([ik_pen(p, arm) for p in p0 + s * (p1 - p0)])


def max_line_deviation(q_path, p0, p1, arm: PenArm = PenArm()) -> float:
    """Largest distance (cm) between the pen path and the straight segment p0-p1."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    pts = np.array([fk_pen(q, arm) for q in q_path])
    t = np.clip(((pts - p0) @ d) / (d @ d), 0.0, 1.0)
    return float(np.max(np.linalg.norm(pts - (p0 + t[:, None] * d), axis=1)))
