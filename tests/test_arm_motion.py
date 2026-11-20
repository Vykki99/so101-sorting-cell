"""Motion profile and move planning in sorter.arm. No motors: robot and IK are fakes."""

import numpy as np
import pytest

import sorter.arm as arm_mod
from sorter.arm import Arm, blend, joint_trajectory, with_gripper
from sorter.workspace import WorkspaceBounds


def test_blend_is_still_at_both_ends_and_reaches_the_goal():
    assert blend(0.0) == 0.0
    assert blend(1.0) == 1.0
    assert blend(0.5) == 0.5
    assert blend(0.1) < 0.1
    assert blend(0.9) > 0.9


def test_joint_trajectory_eases_in_and_out_and_never_reverses_a_joint():
    start = np.array([0.0, -80.0, 90.0, 40.0, -90.0, 50.0])
    goal = start + np.array([2.0, 30.0, -10.0, -15.0, 1.0, 0.0])
    path = joint_trajectory(start, goal, 60)
    steps = np.diff(np.stack([start, *path]), axis=0)
    for joint in steps.T:
        moving = joint[np.abs(joint) > 1e-9]
        assert np.all(np.sign(moving) == np.sign(moving[0])) if moving.size else True
    sizes = np.linalg.norm(steps, axis=1)
    assert sizes[0] < sizes[30] and sizes[-1] < sizes[30]
    np.testing.assert_allclose(path[-1], goal)


def test_with_gripper_sets_or_keeps_and_rejects_out_of_range():
    q = np.zeros(6)
    assert with_gripper(q, None)[5] == 0.0
    assert with_gripper(q, 80)[5] == 80.0
    assert q[5] == 0.0
    with pytest.raises(ValueError):
        with_gripper(q, 101)


class FakeKin:
    """Gripper position is the first three joints, in millimetres."""

    def forward_kinematics(self, q):
        pose = np.eye(4)
        pose[:3, 3] = np.asarray(q[:3]) / 1000.0
        return pose

    def inverse_kinematics(self, q, pose, orientation_weight):
        out = np.array(q, dtype=float)
        out[:3] = pose[:3, 3] * 1000.0
        return out


class FakeRobot:
    def __init__(self, q):
        self.q = np.array(q, dtype=float)
        self.sent = []

    def get_observation(self):
        return {f"{j}.pos": v for j, v in zip(arm_mod.JOINTS, self.q)}

    def send_action(self, action):
        self.sent.append(np.array([action[f"{j}.pos"] for j in arm_mod.JOINTS]))
        return action


@pytest.fixture
def fake_arm(monkeypatch):
    monkeypatch.setattr(arm_mod.time, "sleep", lambda s: None)
    arm = Arm.__new__(Arm)
    arm.robot = FakeRobot([100.0, 0.0, 50.0, 0.0, 0.0, 30.0])
    arm.kin = FakeKin()
    arm.bounds = WorkspaceBounds(0.0, 0.2, -0.1, 0.1, 0.0, 0.1)
    arm._q_cmd = None
    return arm


def test_move_through_streams_each_leg_then_settles(fake_arm):
    fake_arm.move_through([[0.14, 0.0, 0.05], [0.14, 0.04, 0.05]], 1.0)
    sent = fake_arm.robot.sent
    legs = 2 * round(arm_mod.CONTROL_HZ)
    settle = round(arm_mod.SETTLE_S * arm_mod.CONTROL_HZ)
    assert len(sent) == legs + settle
    np.testing.assert_allclose(sent[legs // 2 - 1][:3], [140.0, 0.0, 50.0])
    np.testing.assert_allclose(sent[-1][:3], [140.0, 40.0, 50.0])
    assert all(q[5] == 30.0 for q in sent)


def test_targets_are_clamped_to_the_box(fake_arm):
    fake_arm.move_to([0.5, -0.5, -0.1], 0.5)
    np.testing.assert_allclose(fake_arm.robot.sent[-1][:3], [200.0, -100.0, 0.0])


def test_second_move_starts_from_the_last_command_not_the_encoders(fake_arm):
    fake_arm.move_to([0.12, 0.0, 0.05], 0.5)
    fake_arm.robot.q[:3] = [0.0, 0.0, 0.0]  # encoders lag far behind
    fake_arm.robot.sent.clear()
    fake_arm.move_to([0.12, 0.0, 0.06], 0.5)
    first = fake_arm.robot.sent[0]
    assert np.linalg.norm(first[:3] - [120.0, 0.0, 50.0]) < 1.0


def test_bad_gripper_fails_before_any_command(fake_arm):
    with pytest.raises(ValueError):
        fake_arm.move_to([0.12, 0.0, 0.05], 0.5, gripper=150)
    assert fake_arm.robot.sent == []
