"""Camera-to-robot delta mapping and workspace clamp.

Ported from aruco-panda-servo-teleop teleop_mapping.camera_to_robot and
workspace_limits.WorkspaceBounds. The Panda clamp used one symmetric sideways
limit. This arm's measured box is not symmetric in Y, so each axis has its own
minimum and maximum from config/workcell.yaml.
"""

from __future__ import annotations

from dataclasses import dataclass, fields

import numpy as np
import yaml


def camera_to_robot(delta_cam):
    raise NotImplementedError("camera_to_robot is not implemented")


@dataclass(frozen=True)
class WorkspaceBounds:
    """Per-axis box in the robot base frame, in metres."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    def __post_init__(self):
        if self.x_min >= self.x_max or self.y_min >= self.y_max or self.z_min >= self.z_max:
            raise ValueError("each workspace minimum must be below its maximum")

    def confine(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """Clamp each axis on its own, so a limit on one axis never moves the others."""
        x = max(self.x_min, min(self.x_max, float(x)))
        y = max(self.y_min, min(self.y_max, float(y)))
        z = max(self.z_min, min(self.z_max, float(z)))
        return (x, y, z)

    def clamp(self, xyz) -> np.ndarray:
        return np.array(self.confine(*xyz[:3]), dtype=float)


def load_workspace(path: str = "config/workcell.yaml") -> WorkspaceBounds:
    """Read the measured box from workcell.yaml."""
    with open(path) as fh:
        box = yaml.safe_load(fh)["workspace"]
    return WorkspaceBounds(**{f.name: float(box[f.name]) for f in fields(WorkspaceBounds)})
