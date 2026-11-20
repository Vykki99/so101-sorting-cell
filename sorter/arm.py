"""SO101Follower plus RobotKinematics, with a clamped move_to().

This is the only module that talks to the motors. A move is planned once:
inverse kinematics runs before anything is sent, then the joints are played
back in small steps. Solving again from the live encoders on every tick makes
the servos shake. Targets are clamped to the measured workspace, the
orientation at the start of the move is held, and each joint command is capped
again by max_relative_target.
"""

from __future__ import annotations

import time

import numpy as np
from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from sorter.workspace import WorkspaceBounds, load_workspace

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
JOINTS = ARM_JOINTS + ["gripper"]
CONTROL_HZ = 60.0
# Hardware safety net: the largest step, in degrees, any joint may take per command.
MAX_STEP_DEG = 5.0
# How long to keep sending the final joints so the servo arrives before the next move.
SETTLE_S = 0.25
# LeRobot's own default. Zero lets the 5-DoF wrist fold during a short move.
ORIENTATION_WEIGHT = 0.01
# Straight-line Cartesian waypoints used to walk IK from one point to the next.
IK_SUBSTEPS = 20


def blend(alpha: float) -> float:
    """Smoothstep. Zero slope at 0 and at 1, so a direction change is not a yank."""
    a = min(1.0, max(0.0, float(alpha)))
    return a * a * (3.0 - 2.0 * a)


def joint_trajectory(q_start, q_goal, steps: int) -> list[np.ndarray]:
    """Joint angles from q_start to q_goal. Each joint moves in one direction only."""
    if steps < 1:
        raise ValueError("steps must be positive")
    start = np.asarray(q_start, dtype=float)
    goal = np.asarray(q_goal, dtype=float)
    return [start + (goal - start) * blend(i / steps) for i in range(1, steps + 1)]


def with_gripper(q, gripper) -> np.ndarray:
    """q with its gripper set, 0 (closed) to 100 (open). None keeps the current value."""
    q = np.array(q, dtype=float)
    if gripper is not None:
        value = float(gripper)
        if not 0.0 <= value <= 100.0:
            raise ValueError(f"gripper must be from 0 (closed) to 100 (open), got {value}")
        q[5] = value
    return q


def _pose(xyz, orientation) -> np.ndarray:
    pose = np.eye(4)
    pose[:3, :3] = orientation
    pose[:3, 3] = xyz
    return pose


class Arm:
    def __init__(
        self,
        port,
        urdf="urdf/so101_new_calib.urdf",
        robot_id="follower_arm",
        bounds: WorkspaceBounds | None = None,
    ):
        self.robot = SO101Follower(
            SO101FollowerConfig(port=port, id=robot_id, max_relative_target=MAX_STEP_DEG)
        )
        self.kin = RobotKinematics(
            urdf, target_frame_name="gripper_frame_link", joint_names=ARM_JOINTS
        )
        self.bounds = load_workspace() if bounds is None else bounds
        self._q_cmd = None

    def connect(self):
        self._q_cmd = None
        self.robot.connect()

    def disconnect(self):
        self._q_cmd = None
        self.robot.disconnect()

    def joints(self) -> np.ndarray:
        """Degrees for the five arm joints, then gripper (0 closed, 100 open)."""
        obs = self.robot.get_observation()
        return np.array([obs[f"{j}.pos"] for j in JOINTS], dtype=float)

    def ee_pose(self) -> np.ndarray:
        """4x4 end-effector pose in metres, robot base frame, from the encoders."""
        return self._fk(self.joints())

    def step_toward(self, target_xyz, gripper=None, orientation=None):
        """One inverse-kinematics step toward a clamped Cartesian target."""
        q = self._start_joints()
        if orientation is None:
            orientation = self._fk(q)[:3, :3]
        pose = _pose(self.bounds.clamp(target_xyz), orientation)
        q_new = self.kin.inverse_kinematics(q, pose, orientation_weight=ORIENTATION_WEIGHT)
        self._command(with_gripper(q_new, gripper))

    def move_to(self, xyz, duration_s, gripper=None):
        """Ease from the current end-effector position to xyz at CONTROL_HZ."""
        self.move_through([xyz], duration_s, gripper=gripper)

    def move_through(self, points, duration_s, gripper=None):
        """Visit each Cartesian point in order, at duration_s per point.

        The joint path for the whole sequence is planned before the first
        command, then streamed. Corners are not re-solved from the encoders.
        The gripper, if given, eases to its value over the first point.
        """
        if duration_s <= 0:
            raise ValueError("duration_s must be positive")
        if len(points) == 0:
            raise ValueError("points must not be empty")
        q = self._start_joints()
        orientation = self._fk(q)[:3, :3]
        planned = [q]
        for point in points:
            planned.append(with_gripper(self._ik_to(planned[-1], point, orientation), gripper))

        steps = max(1, round(duration_s * CONTROL_HZ))
        path = [
            q_step
            for q_start, q_goal in zip(planned, planned[1:])
            for q_step in joint_trajectory(q_start, q_goal, steps)
        ]
        path += [planned[-1]] * max(1, round(SETTLE_S * CONTROL_HZ))
        self._stream(path)

    def _start_joints(self) -> np.ndarray:
        # Continue from the last command, not the encoders: re-reading them
        # between moves would solve servo lag and gravity sag into the start.
        return self.joints() if self._q_cmd is None else self._q_cmd.copy()

    def _fk(self, q) -> np.ndarray:
        return np.array(self.kin.forward_kinematics(q), dtype=float)

    def _ik_to(self, q, xyz, orientation) -> np.ndarray:
        """Walk inverse kinematics along a straight line to xyz. Sends nothing."""
        q = np.asarray(q, dtype=float)
        start = self._fk(q)[:3, 3]
        goal = self.bounds.clamp(xyz)
        for i in range(1, IK_SUBSTEPS + 1):
            waypoint = self.bounds.clamp(start + (goal - start) * (i / IK_SUBSTEPS))
            q = np.asarray(
                self.kin.inverse_kinematics(
                    q, _pose(waypoint, orientation), orientation_weight=ORIENTATION_WEIGHT
                ),
                dtype=float,
            )
        return q

    def _command(self, q):
        action = {f"{j}.pos": float(v) for j, v in zip(JOINTS, q)}
        sent = self.robot.send_action(action)
        self._q_cmd = np.array([sent[f"{j}.pos"] for j in JOINTS], dtype=float)

    def _stream(self, path):
        """Send each joint target on its own CONTROL_HZ tick (monotonic clock)."""
        period = 1.0 / CONTROL_HZ
        for q in path:
            tick = time.monotonic()
            self._command(q)
            remaining = period - (time.monotonic() - tick)
            if remaining > 0:
                time.sleep(remaining)
