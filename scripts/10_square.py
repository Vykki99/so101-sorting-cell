"""Trace a 4 cm square, 3 cm above the gripper's starting height.

No camera. Run from anywhere, with the follower powered and a hand near the
power plug. The script prints the path and waits for Enter before moving.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)

import numpy as np

from sorter.arm import Arm

SIDE_M = 0.04
LIFT_M = 0.03
SIDE_S = 2.0


def _fmt(xyz) -> str:
    return f"x={xyz[0] * 1000:7.1f}  y={xyz[1] * 1000:7.1f}  z={xyz[2] * 1000:7.1f} mm"


def square_corners(start_xyz) -> list[tuple[str, np.ndarray]]:
    """Raised start, then a 4 cm square that closes back on that raised point."""
    raised = np.asarray(start_xyz, dtype=float) + [0.0, 0.0, LIFT_M]
    return [
        ("lift 3 cm", raised),
        ("+4 cm in x", raised + [SIDE_M, 0.0, 0.0]),
        ("+4 cm in y", raised + [SIDE_M, SIDE_M, 0.0]),
        ("-4 cm in x", raised + [0.0, SIDE_M, 0.0]),
        ("-4 cm in y (close)", raised),
    ]


def main() -> None:
    port = os.environ.get("FOLLOWER")
    if not port:
        sys.exit("FOLLOWER is not set. Export it, for example: export FOLLOWER=/dev/so101_follower")

    arm = Arm(port)
    try:
        arm.connect()
        start = arm.ee_pose()[:3, 3]
        print(f"Start: {_fmt(start)}")
        goals, clipped = [], False
        for label, corner in square_corners(start):
            goal = arm.bounds.clamp(corner)
            goals.append(goal)
            inside = np.allclose(corner, goal, atol=1e-6)
            clipped |= not inside
            print(f"  {label:24} {_fmt(goal)}{'' if inside else '  CLAMPED'}")
        if clipped:
            print("A corner is outside the measured box, so the square will be flattened.")
            print("Ctrl+C, pose the gripper nearer the middle of the table, and run this again.")
            input("Or press Enter to move the flattened path anyway.\n")
        else:
            input("Press Enter to move. Ctrl+C aborts. Keep a hand near the power plug.\n")
        arm.move_through(goals, SIDE_S)
        done = arm.ee_pose()[:3, 3]
        print(f"Finish: {_fmt(done)}")
        print(f"Distance from the raised start: {np.linalg.norm(done - goals[-1]) * 1000:.1f} mm")
        input("Press Enter to release torque.\n")
    finally:
        if arm.robot.is_connected:
            arm.disconnect()


if __name__ == "__main__":
    main()
