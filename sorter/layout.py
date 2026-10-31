"""Bin layout and cube-in-bin checks.

Ported from virtual-sorter workcell_layout.py. Positions are in the robot base frame.
"""


def load_layout(path="config/workcell.yaml"):
    raise NotImplementedError("load_layout is not implemented")


def cube_in_bin(cube_xyz, bin_xyz):
    raise NotImplementedError("cube_in_bin is not implemented")
