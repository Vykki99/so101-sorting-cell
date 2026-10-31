"""Camera intrinsics and pixel-to-base projection.

Ported from aruco-panda-servo-teleop camera_calibration.py.
"""


def load_intrinsics(path="config/camera_intrinsics.yaml"):
    raise NotImplementedError("load_intrinsics is not implemented")


def pixel_to_base(u, v, z_plane):
    raise NotImplementedError("pixel_to_base is not implemented")
