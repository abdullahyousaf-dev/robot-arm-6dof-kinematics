import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from arm import (  # noqa: E402
    PenArm,
    UnreachableError,
    cartesian_segment,
    fk_pen,
    forward_kinematics_dh,
    ik_pen,
    max_line_deviation,
    quintic_coefficients,
)


def test_ik_matches_angles_sent_to_servos():
    # Angles recorded in firmware/quintic_spline for point (0, 15, -5).
    q = ik_pen((0, 15, -5))
    assert np.allclose(q, [90.0, 47.68, 105.46], atol=0.01)


@pytest.mark.parametrize("seed", range(5))
def test_ik_fk_round_trip(seed):
    rng = np.random.default_rng(seed)
    arm = PenArm()
    for _ in range(200):
        r = rng.uniform(8.0, 24.0)
        az = rng.uniform(-np.pi, np.pi)
        z = rng.uniform(-8.0, 8.0)
        p = np.array([r * np.cos(az), r * np.sin(az), z])
        if np.linalg.norm(p) > sum((arm.l2, arm.l3)) - 0.1:
            continue
        assert np.allclose(fk_pen(ik_pen(p, arm), arm), p, atol=1e-9)


def test_unreachable_point_raises():
    with pytest.raises(UnreachableError):
        ik_pen((0, 40, 0))


def test_quintic_boundary_conditions():
    c = quintic_coefficients(10.0, 80.0, T=4.0)
    t = 4.0
    pos = sum(ck * t**k for k, ck in enumerate(c))
    vel = sum(k * ck * t ** (k - 1) for k, ck in enumerate(c) if k)
    assert c[0] == pytest.approx(10.0)
    assert c[1] == pytest.approx(0.0) and c[2] == pytest.approx(0.0)
    assert pos == pytest.approx(80.0) and vel == pytest.approx(0.0, abs=1e-9)


def test_cartesian_stroke_is_straight():
    p0, p1 = (0, 15, -5), (5, 22, -5)
    assert max_line_deviation(cartesian_segment(p0, p1, 50), p0, p1) < 1e-9


def test_dh_home_pose_is_finite():
    T = forward_kinematics_dh([0, 0, 0, 0, 0, 0])
    assert T.shape == (4, 4) and np.all(np.isfinite(T))
    assert np.allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-12)
