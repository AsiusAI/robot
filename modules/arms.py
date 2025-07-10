import dataclasses
import numpy as np
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.utils.robot_utils import busy_wait
from lerobot.model.kinematics import RobotKinematics
from robots.v1 import Arm, ArmPosition

# Define arms
arms = {
    "left": SO101Follower(SO101FollowerConfig(port="/dev/ttyACM0", id="left", use_degrees= True)),
    "right": SO101Follower(SO101FollowerConfig(port="/dev/ttyACM1", id="right", use_degrees= True)),
}
connected = False

def connect_arms():
    global connected
    try:
        for _, arm in arms.items():
            arm.connect(False)
        connected = True
        print("Arms connected.")
    except Exception as e:
        print(f"Failed to connect arms: {e}")
        connected = False

def get_arm_pos(arm):
    if not connected: raise Exception("Not connected")
    arm = arms[arm]
    return arm.get_observation()


def move_arm(arm:Arm, pos:ArmPosition):
    if not connected: raise Exception("Not connected")
    arm = arms[arm]
    arm.send_action(dataclasses.asdict(pos))


def disconnect_arms():
    global connected
    if not connected: return
    for _, arm in arms.items():
        arm.disconnect()
    connected = False
    print("Arms disconnected.")
