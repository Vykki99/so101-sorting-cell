"""Axis-preserving workspace clamp, with the measured SO-101 box."""

import numpy as np

from sorter.workspace import WorkspaceBounds, load_workspace

# 2026-11-06 sweep, shrunk by 2 cm on every side.
MEASURED = WorkspaceBounds(
    x_min=-0.0020,
    x_max=0.3438,
    y_min=-0.2866,
    y_max=0.3246,
    z_min=0.0005,
    z_max=0.2417,
)


def test_motion_on_one_axis_leaves_the_others_alone():
    x, y, z = 0.15, 0.0, 0.10

    forward = MEASURED.confine(0.25, y, z)
    left = MEASURED.confine(x, -0.10, z)
    up = MEASURED.confine(x, y, 0.20)

    assert forward[1:] == (y, z)
    assert forward[0] > x
    assert (left[0], left[2]) == (x, z)
    assert left[1] < y
    assert (up[0], up[1]) == (x, y)
    assert up[2] > z


def test_each_face_saturates_without_moving_the_other_axes():
    inside = (0.15, 0.0, 0.10)

    past_x = MEASURED.confine(MEASURED.x_max + 0.05, *inside[1:])
    past_y_min = MEASURED.confine(inside[0], MEASURED.y_min - 0.05, inside[2])
    past_y_max = MEASURED.confine(inside[0], MEASURED.y_max + 0.05, inside[2])
    past_z = MEASURED.confine(*inside[:2], MEASURED.z_min - 0.05)

    assert past_x == (MEASURED.x_max, inside[1], inside[2])
    assert past_y_min == (inside[0], MEASURED.y_min, inside[2])
    assert past_y_max == (inside[0], MEASURED.y_max, inside[2])
    assert past_z == (inside[0], inside[1], MEASURED.z_min)


def test_point_inside_the_box_is_unchanged():
    point = (0.15, -0.04, 0.08)
    assert MEASURED.confine(*point) == point


def test_y_limits_are_not_mirrors_of_each_other():
    assert MEASURED.y_min != -MEASURED.y_max


def test_clamp_returns_the_confined_point_as_an_array():
    clamped = MEASURED.clamp([1.0, -1.0, 0.10])
    assert isinstance(clamped, np.ndarray)
    assert clamped.shape == (3,)
    np.testing.assert_allclose(clamped, [MEASURED.x_max, MEASURED.y_min, 0.10])


def test_workcell_yaml_loads_the_measured_box():
    loaded = load_workspace("config/workcell.yaml")
    assert loaded == MEASURED
