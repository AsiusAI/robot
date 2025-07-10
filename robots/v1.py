from typing import Literal, Optional

from aiortc import MediaStreamTrack
from modules.arms import connect_arms, disconnect_arms, move_arm
from modules.camera import create_local_tracks, stop_camera
from modules.wheels import cleanup_motors, get_status, move, start_odrive
from robots import ArmPosition, Robot


class RobotV1(Robot):
    def start(self):
        start_odrive()
        connect_arms()

    def stop(self):
        cleanup_motors()
        disconnect_arms()
        stop_camera()

    def move(self, left, right, speed):
        move(left=left, right=right, speed=speed)

    def move_arm(self, arm: Literal["left", "right"], pos: ArmPosition):
        move_arm(arm, pos)

    def status(self):
        return get_status()

    def get_media_stream(
        self,
    ) -> tuple[Optional[MediaStreamTrack], Optional[MediaStreamTrack]]:
        return create_local_tracks()
