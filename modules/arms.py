from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.utils.robot_utils import busy_wait

# Define the robot config
arm = SO101Follower(SO101FollowerConfig(port="/dev/ttyACM0", id="left"))
# arm = SO101Follower(SO101FollowerConfig(port="/dev/ttyACM1", id="right"))

arm.connect(False)
print(arm.get_observation())
arm.send_action(
    {
        "shoulder_pan.pos": 0,
        "shoulder_lift.pos": 0,
        "elbow_flex.pos": 0,
        "wrist_flex.pos": 0,
        "wrist_roll.pos": 0,
        "gripper.pos": 50,
    }
)
busy_wait(10)

arm.disconnect()
