"""Forward and inverse kinematics for a 6-DOF desktop arm driven by MG996R servos.

Two models are used:

* ``forward_kinematics_dh`` - full 6-joint chain with modified (Craig) DH
  parameters measured on the physical arm. Used to check where the tool ends up.
* ``ik_pen`` / ``fk_pen`` - the 3-joint positional model (base yaw, shoulder,
  elbow) used for writing. The wrist servo is held fixed, so the pen tip only
  depends on these three joints. This is the model the Arduino firmware runs.

All lengths are in centimetres, all public angles in degrees.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Modified DH parameters (a_{i-1}, alpha_{i-1}, d_i), measured on the arm.
DH_A = np.array([0.0, 0.0, 10.5, 10.0, 2.0, 0.0])
DH_ALPHA_DEG = np.array([0.0, 90.0, 0.0, 0.0, -90.0, 0.0])
DH_D = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 17.0])
# Servo zero vs. DH zero, found during calibration. Joint 4 is mounted reversed.
JOINT_OFFSETS_DEG = np.array([5.0, -2.0, -3.0, 0.0, 0.0, 0.0])
JOINT_SIGNS = np.array([1, 1, 1, -1, 1, 1])


def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """Modified DH link transform (angles in radians)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -sa * d],
            [st * sa, ct * sa, ca, ca * d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def forward_kinematics_dh(servo_deg) -> np.ndarray:
    """Return the 4x4 base-to-tool transform for six servo angles (degrees)."""
    servo_deg = np.asarray(servo_deg, dtype=float)
    if servo_deg.shape != (6,):
        raise ValueError("expected 6 joint angles")
    theta = np.deg2rad(JOINT_SIGNS * servo_deg + JOINT_OFFSETS_DEG)
    alpha = np.deg2rad(DH_ALPHA_DEG)
    T = np.eye(4)
    for i in range(6):
        T = T @ dh_transform(DH_A[i], alpha[i], DH_D[i], theta[i])
    return T


def in_safe_workspace(p) -> bool:
    """Workspace limits used on the real arm: stay above the table and keep
    clear of the column around the base."""
    x, y, z = p
    if z <= 2.0:
        return False
    if -5.0 <= x <= 5.0 and -5.0 <= y <= 5.0 and z <= 15.0:
        return False
    return True


@dataclass(frozen=True)
class PenArm:
    """Positional model used for writing: upper arm l2, forearm + pen l3.

    l3 = 15 cm matches the joint angles that were actually sent to the servos
    (see firmware/quintic_spline). Some sketches declare l3 = 12, but with that
    value the top of the letters is out of reach."""

    l2: float = 10.4
    l3: float = 15.0

    @property
    def reach(self) -> tuple[float, float]:
        return abs(self.l2 - self.l3), self.l2 + self.l3


class UnreachableError(ValueError):
    pass


def ik_pen(p, arm: PenArm = PenArm()) -> np.ndarray:
    """Closed-form IK for the pen tip. Returns [base, shoulder, elbow] in degrees.

    base     = atan2(y, x)
    elbow    from the law of cosines on |p|
    shoulder solves  a*cos(s) + b*sin(s) = z  with a = -l3 sin(e), b = l2 + l3 cos(e)
    """
    x, y, z = map(float, p)
    d2 = x * x + y * y + z * z
    lo, hi = arm.reach
    if not (lo**2 - 1e-9 <= d2 <= hi**2 + 1e-9):
        raise UnreachableError(f"point {p} is outside reach [{lo:.1f}, {hi:.1f}] cm")

    base = np.arctan2(y, x)
    c_e = np.clip((d2 - arm.l2**2 - arm.l3**2) / (2 * arm.l2 * arm.l3), -1.0, 1.0)
    elbow = np.arctan2(np.sqrt(1.0 - c_e**2), c_e)

    a = -arm.l3 * np.sin(elbow)
    b = arm.l2 + arm.l3 * np.cos(elbow)
    root = np.sqrt(max(a * a + b * b - z * z, 0.0))
    shoulder = np.arctan2(z, root) - np.arctan2(a, b)
    return np.rad2deg([base, shoulder, elbow])


def fk_pen(q_deg, arm: PenArm = PenArm()) -> np.ndarray:
    """Pen-tip position for [base, shoulder, elbow] in degrees."""
    base, shoulder, elbow = np.deg2rad(np.asarray(q_deg, dtype=float))
    r = arm.l2 * np.cos(shoulder) + arm.l3 * np.cos(shoulder - elbow)
    z = arm.l2 * np.sin(shoulder) + arm.l3 * np.sin(shoulder - elbow)
    return np.array([np.cos(base) * r, np.sin(base) * r, z])


def pen_joint_positions(q_deg, arm: PenArm = PenArm()) -> np.ndarray:
    """Base, elbow-joint and pen-tip positions (3x3), for plotting the arm."""
    base, shoulder, _ = np.deg2rad(np.asarray(q_deg, dtype=float))
    elbow_pt = arm.l2 * np.array(
        [np.cos(base) * np.cos(shoulder), np.sin(base) * np.cos(shoulder), np.sin(shoulder)]
    )
    return np.vstack([np.zeros(3), elbow_pt, fk_pen(q_deg, arm)])
