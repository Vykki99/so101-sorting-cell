"""ArUco detection, solvePnP, and EMA smoothing.

Ported from opencv-3d-spatial-tracker tracker.py.
"""


class ArucoTracker:
    def detect(self, frame):
        """Return (rvec, tvec) or None."""
        raise NotImplementedError("ArucoTracker.detect is not implemented")
