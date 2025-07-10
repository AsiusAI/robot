from dataclasses import dataclass
from typing import List, Literal, Optional, Union

import placo
import numpy as np
import placo
from scipy.spatial.transform import Rotation


@dataclass
class Status:
    voltage: float
    current: float
    uptime: int


@dataclass
class ArmPosition:
    shoulder_pan: Optional[float] = None
    shoulder_lift: Optional[float] = None
    elbow_flex: Optional[float] = None
    wrist_flex: Optional[float] = None
    wrist_roll: Optional[float] = None
    gripper: Optional[float] = None


robot = placo.RobotWrapper(
    "sim/SO101/so101_new_calib.urdf",
)
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)
effector_task = solver.add_frame_task("gripper_frame_joint", np.eye(4))
effector_task.configure("effector", "soft", 10.0, 1.0)
solver.enable_velocity_limits(True)
solver.dt = 0.01


class Robot:
    def start(self):
        raise Exception("Not implemented!")

    def stop(self):
        raise Exception("Not implemented!")

    def move(self, left: float, right: float, speed: float):
        raise Exception("Not implemented!")

    def status(self):
        raise Exception("Not implemented!")

    def move_with_joystick(self, x: float, y: float, speed: float):
        left = y + x
        right = y - x

        left = max(-1, min(1, left))
        right = max(-1, min(1, right))

        self.move(left=left, right=right, speed=speed)

    def move_arm(self, arm: Literal["left", "right"], pos: ArmPosition):
        raise Exception("Not implemented!")

    def _get_ik(
        self,
        arm: Literal["left", "right"],
        position: List[float],
        orientation: List[float],
    ):
        x, y, z = position
        if arm == "left":
            position = [x, z * -1, y * -1]
        if arm == "right":
            position = [x, z, y]

        T_world_frame = np.eye(4)
        rotation = Rotation.from_quat(orientation)
        T_world_frame[:3, :3] = rotation.as_matrix()
        T_world_frame[:3, 3] = position
        effector_task.T_world_frame = T_world_frame
        solver.solve(True)
        robot.update_kinematics()
        return ArmPosition(*robot.state.q[7:])

    def move_arm_with_ik(
        self,
        arm: Literal["left", "right"],
        position: List[float],
        orientation: List[float],
    ):
        pos = self._get_ik(arm, position, orientation)
        return self.move_arm(arm, pos)
