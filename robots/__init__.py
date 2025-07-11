from dataclasses import dataclass
from typing import List, Literal, Optional, Union

from aiortc import MediaStreamTrack
import placo
import numpy as np
import placo
from scipy.spatial.transform import Rotation as R


@dataclass
class Status:
    voltage: float
    current: float
    uptime: int


@dataclass
class ArmPosition:
    position: List[float]
    orientation: List[float]
    shoulder_pan: Optional[float] = None
    shoulder_lift: Optional[float] = None
    elbow_flex: Optional[float] = None
    wrist_flex: Optional[float] = None
    wrist_roll: Optional[float] = None
    gripper: Optional[float] = None


left_robot = placo.RobotWrapper(
    "sim/SO101/so101_new_calib.urdf",
)
left_solver = placo.KinematicsSolver(left_robot)
left_solver.mask_fbase(True)
left_task = left_solver.add_frame_task("gripper_frame_link", np.eye(4))
left_task.configure("gripper_frame_link", "soft", 1.0, 0.01)
left_solver.enable_velocity_limits(True)
left_solver.dt = 0.01

right_robot = placo.RobotWrapper(
    "sim/SO101/so101_new_calib.urdf",
)
right_solver = placo.KinematicsSolver(right_robot)
right_solver.mask_fbase(True)
right_task = right_solver.add_frame_task("gripper_frame_link", np.eye(4))
right_task.configure("gripper_frame_link", "soft", 1.0, 0.01)
right_solver.enable_velocity_limits(True)
right_solver.dt = 0.01


class Robot:
    def start(self):
        raise Exception("start() not implemented!")

    def stop(self):
        raise Exception("stop() not implemented!")

    def status(self):
        raise Exception("status() not implemented!")

    def move(self, left: float, right: float, speed: float):
        raise Exception("move() not implemented!")

    def get_media_stream(
        self,
    ) -> tuple[Optional[MediaStreamTrack], Optional[MediaStreamTrack]]:
        raise Exception("get_media_stream() not implemented!")

    def move_arm(self, arm: Literal["left", "right"], pos: ArmPosition):
        raise Exception("move_arm() not implemented!")

    def move_with_joystick(self, x: float, y: float, speed: float):
        left = y + x
        right = y - x

        left = max(-1, min(1, left))
        right = max(-1, min(1, right))

        self.move(left=left, right=right, speed=speed)

    def _get_ik(
        self,
        arm: Literal["left", "right"],
        position: List[float],
        orientation: List[float],
    ):
        # Converting from WebXR to pybullet coordinate system
        pX, pY, pZ = position
        position = [-pZ, -pX, pY]

        rotation = R.from_quat([0.5, -0.5, 0.5, -0.5]) * R.from_quat(orientation)
        orientation = rotation.as_quat()

        T_world_frame = np.eye(4)
        T_world_frame[:3, :3] = rotation.as_matrix()
        T_world_frame[:3, 3] = position
        if arm == "left":
            left_task.T_world_frame = T_world_frame
            left_solver.solve(True)
            left_robot.update_kinematics()
            return ArmPosition(*[position, orientation, *left_robot.state.q[7:]])
        if arm == "right":
            right_task.T_world_frame = T_world_frame
            right_solver.solve(True)
            right_robot.update_kinematics()
            return ArmPosition(*[position, orientation, *right_robot.state.q[7:]])

    def move_arm_with_ik(
        self,
        arm: Literal["left", "right"],
        position: List[float],
        orientation: List[float],
    ):
        pos = self._get_ik(arm, position, orientation)
        return self.move_arm(arm, pos)
