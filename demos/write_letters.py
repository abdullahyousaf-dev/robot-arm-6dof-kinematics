"""Simulate the arm writing N, P and T, and compare two ways of moving between points.

    python demos/write_letters.py            # saves figures to media/
    python demos/write_letters.py --show     # also opens the plots
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from arm import (  # noqa: E402
    PenArm,
    cartesian_segment,
    fk_pen,
    joint_space_segment,
    max_line_deviation,
    pen_joint_positions,
)

# Paper plane: x across the page, y away from the robot, z = table height (cm).
Z_PAPER = -5.0
LIFT = 2.0
LETTERS = {
    # Same corner points as firmware/letter_n.
    "N": [[(0, 15), (0, 22), (5, 15), (5, 22)]],
    "T": [[(0, 22), (5, 22)], [(2.5, 22), (2.5, 15)]],
    "P": [[(0, 15), (0, 22), (3.5, 22), (5, 20.5), (3.5, 19), (0, 19)]],
}
LETTER_SPACING = 7.0


def letter_strokes(letter: str, x_shift: float):
    for stroke in LETTERS[letter]:
        yield [(x + x_shift, y, Z_PAPER) for x, y in stroke]


def plan(arm: PenArm, n=60):
    """Return the joint path for all letters plus per-stroke deviation numbers."""
    path, report = [], []
    for i, letter in enumerate(LETTERS):
        for stroke in letter_strokes(letter, i * LETTER_SPACING - 7.0):
            if path:  # pen up, travel, pen down
                last = fk_pen(path[-1], arm)
                up = (*last[:2], Z_PAPER + LIFT)
                above = (*stroke[0][:2], Z_PAPER + LIFT)
                for a, b in [(last, up), (up, above), (above, stroke[0])]:
                    path.extend(cartesian_segment(a, b, n // 2, arm))
            for p0, p1 in zip(stroke, stroke[1:]):
                joint = joint_space_segment(p0, p1, n, arm)
                cart = cartesian_segment(p0, p1, n, arm)
                report.append(
                    (letter, max_line_deviation(joint, p0, p1, arm), max_line_deviation(cart, p0, p1, arm))
                )
                path.extend(cart)
    return path, report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    arm = PenArm()
    path, report = plan(arm)
    pts = np.array([fk_pen(q, arm) for q in path])

    print(f"{'letter':<7}{'joint-space dev (cm)':>22}{'cartesian dev (cm)':>21}")
    for letter, dj, dc in report:
        print(f"{letter:<7}{dj:>22.3f}{dc:>21.2e}")
    worst = max(r[1] for r in report)
    print(f"\nWorst bow with joint-space interpolation: {worst:.2f} cm")
    print("Cartesian interpolation keeps every stroke on its line (error ~0).")

    media = ROOT / "media"
    media.mkdir(exist_ok=True)

    fig = plt.figure(figsize=(11, 5))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    ax.plot(*pts.T, lw=1, color="tab:blue", label="pen tip")
    for q in path[:: max(1, len(path) // 8)]:
        j = pen_joint_positions(q, arm)
        ax.plot(*j.T, "-o", color="0.4", ms=3, lw=2, alpha=0.6)
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_zlabel("z (cm)")
    ax.set_title("Arm poses while writing")
    ax.legend(loc="upper left")

    ax2 = fig.add_subplot(1, 2, 2)
    on_paper = pts[:, 2] < Z_PAPER + 0.05
    ax2.plot(pts[~on_paper, 0], pts[~on_paper, 1], ".", ms=1, color="tab:orange", label="pen up")
    ax2.scatter(pts[on_paper, 0], pts[on_paper, 1], s=2, color="black", label="ink", zorder=3)
    for i, letter in enumerate(LETTERS):
        for stroke in letter_strokes(letter, i * LETTER_SPACING - 7.0):
            for p0, p1 in zip(stroke, stroke[1:]):
                q = joint_space_segment(p0, p1, 60, arm)
                bow = np.array([fk_pen(k, arm) for k in q])
                ax2.plot(bow[:, 0], bow[:, 1], "--", lw=1, color="tab:red", alpha=0.7)
    ax2.plot([], [], "--", color="tab:red", label="joint-space path")
    ax2.set_aspect("equal")
    ax2.set_xlabel("x (cm)")
    ax2.set_ylabel("y (cm)")
    ax2.set_title("Top view: written letters")
    ax2.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    out = media / "letters_sim.png"
    fig.savefig(out, dpi=130)
    print(f"Saved {out.relative_to(ROOT)}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
