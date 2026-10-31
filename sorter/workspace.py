"""Camera-to-robot delta mapping and workspace clamp.

Ported from aruco-panda-servo-teleop teleop_mapping.camera_to_robot and
workspace_limits.WorkspaceBounds.
"""


def camera_to_robot(delta_cam):
    raise NotImplementedError("camera_to_robot is not implemented")


class WorkspaceBounds:
    def clamp(self, xyz):
        raise NotImplementedError("WorkspaceBounds.clamp is not implemented")
