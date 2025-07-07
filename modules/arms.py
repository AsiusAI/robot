from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.utils.robot_utils import busy_wait

arms = {
    "left": SO101Follower(SO101FollowerConfig(port="/dev/ttyACM0", id="left")),
    "right": SO101Follower(SO101FollowerConfig(port="/dev/ttyACM1", id="right")),
}


def connect_arms():
    for _, arm in arms.items():
        arm.connect(False)


def get_arm_pos(arm):
    arm = arms[arm]
    return arm.get_observation()


def set_gripper_pos(arm, pos):
    arm = arms[arm]
    arm.send_action({"gripper.pos": pos * 100})


# arm.send_action(
#     {
#         "shoulder_pan.pos": 0,
#         "shoulder_lift.pos": 0,
#         "elbow_flex.pos": 0,
#         "wrist_flex.pos": 0,
#         "wrist_roll.pos": 0,
#         "gripper.pos": 50,
#     }
# )
def disconnect_arms():
    for _, arm in arms.items():
        arm.disconnect()
