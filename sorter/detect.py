"""HSV cube detector.

Thresholds live in config/workcell.yaml.
"""


def detect_cubes(frame):
    """Return detections as centroid pixel, colour, and angle."""
    raise NotImplementedError("detect_cubes is not implemented")
