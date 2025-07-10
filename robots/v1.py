from modules.arms import connect_arms, disconnect_arms, move_arm
from modules.wheels import cleanup_motors, get_status, move, start_odrive
from robots import Arm, ArmPosition, Robot


class RobotV1(Robot):
    def start(self):
        start_odrive()
        connect_arms()

    def stop(self):
        cleanup_motors()
        disconnect_arms()

    def move(self, left, right, speed):
        move(left=left, right=right, speed=speed)

    def move_arm(self, arm: Arm, pos: ArmPosition):
        move_arm(arm, pos)

    def status(self):
        return get_status()
