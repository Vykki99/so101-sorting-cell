"""SO101Follower plus RobotKinematics, with a clamped move_to().

This is the only module that talks to the motors. Deadband, slew limiting, and
the stale-target stop are ported from aruco-panda-servo-teleop
pose_to_servo_bridge.py.
"""


class Arm:
    def connect(self):
        raise NotImplementedError("Arm.connect is not implemented")

    def joints(self):
        raise NotImplementedError("Arm.joints is not implemented")

    def ee_pose(self):
        raise NotImplementedError("Arm.ee_pose is not implemented")

    def step_toward(self, target_xyz, gripper=None, orientation_weight=0.0):
        raise NotImplementedError("Arm.step_toward is not implemented")

    def move_to(self, xyz, duration_s, gripper=None):
        raise NotImplementedError("Arm.move_to is not implemented")
