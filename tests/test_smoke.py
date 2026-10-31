import importlib

MODULES = [
    "sorter",
    "sorter.camera",
    "sorter.aruco",
    "sorter.workspace",
    "sorter.layout",
    "sorter.arm",
    "sorter.detect",
    "sorter.pick_place",
]


def test_sorter_modules_import():
    for name in MODULES:
        assert importlib.import_module(name) is not None
