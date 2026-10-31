"""Pick-and-place state machine.

Ported from virtual-sorter pick_place_node.py. States stay approach, descend,
grasp, lift and verify, place, home, and recover. Motion goes through sorter.arm.
"""


class PickPlace:
    def step(self):
        raise NotImplementedError("PickPlace.step is not implemented")
