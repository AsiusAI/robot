import numpy as np
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.utils.robot_utils import busy_wait
from lerobot.model.kinematics import RobotKinematics

# Define arms
arms = {
    "left": SO101Follower(SO101FollowerConfig(port="/dev/ttyACM0", id="left")),
    "right": SO101Follower(SO101FollowerConfig(port="/dev/ttyACM1", id="right")),
}
connected = False

# Define constants for kinematics
ARM_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
TARGET_FRAME = "gripper_frame_link"
URDF_PATH = "sim/SO101/so101_new_calib.urdf"

# Initialize the IK solver once
print("Initializing kinematics solver...")
ik_solver = RobotKinematics(
    urdf_path=URDF_PATH,
    target_frame_name=TARGET_FRAME,
    joint_names=ARM_JOINT_NAMES
)
print("Solver initialized successfully.")

# --- Arm Control Functions ---

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


def set_gripper_pos(arm, pos):
    if not connected: raise Exception("Not connected")
    arm = arms[arm]
    arm.send_action({"gripper.pos": pos * 100})



def disconnect_arms():
    global connected
    if not connected: return
    for _, arm in arms.items():
        arm.disconnect()
    connected = False
    print("Arms disconnected.")

def move_arm_to_pose(
    arm_side: str,
    desired_ee_pose: np.ndarray,
    position_weight: float = 1.0,
    orientation_weight: float = 0.1,
):
    """
    Calculates and moves a robot arm to a desired end-effector pose.

    Args:
        arm_side: The arm to control, either "left" or "right".
        desired_ee_pose: The target 4x4 transformation matrix (numpy array).
        position_weight: The weight for the position objective in the IK solver.
        orientation_weight: The weight for the orientation objective in the IK solver.
    """
    if not connected:
        raise Exception("Arms are not connected.")

    # 1. Get the current state of the robot arm
    arm = arms[arm_side]
    observation = arm.get_observation()
    
    # Create the current joint position array for the IK solver
    # The order must match ARM_JOINT_NAMES, with gripper appended
    current_joint_pos_deg = np.array(
        [observation[f"{name}.pos"] for name in ARM_JOINT_NAMES] +
        [observation["gripper.pos"] / 100.0]  # Gripper from 0-100 to 0-1
    )

    # 2. Solve for the new joint angles using Inverse Kinematics
    print(f"\nMoving '{arm_side}' arm to desired pose...")
    new_joint_angles_deg = ik_solver.inverse_kinematics(
        current_joint_pos=current_joint_pos_deg,
        desired_ee_pose=desired_ee_pose,
        position_weight=position_weight,
        orientation_weight=orientation_weight,
    )
    print(f"Calculated target angles (deg): {np.round(new_joint_angles_deg, 2)}")

    # 3. Create the action dictionary and command the robot
    action_dict = {
        f"{name}.pos": angle for name, angle in zip(ARM_JOINT_NAMES, new_joint_angles_deg)
    }
    # The gripper value was preserved by IK; scale it back to 0-100 for the action
    action_dict["gripper.pos"] = new_joint_angles_deg[-1] * 100.0
    
    arm.send_action(action_dict)
    
    # Wait for the robot to reach the target
    busy_wait(2.0)
    print(f"'{arm_side}' arm movement complete.")


# --- Main Execution Example ---
if __name__ == "__main__":
    try:
        connect_arms()
        
        if connected:
            # Get the current pose of the right arm to use as a starting point
            right_arm = arms["right"]
            obs = right_arm.get_observation()
            start_joints = np.array([obs[f"{name}.pos"] for name in ARM_JOINT_NAMES])
            
            print("\nCalculating current pose of the right arm...")
            current_pose = ik_solver.forward_kinematics(start_joints)
            print(f"Current Pose:\n{np.round(current_pose, 3)}")
            
            # Define a new target pose by moving 5cm forward (X) and 2cm up (Z)
            target_pose = np.copy(current_pose)
            target_pose[0, 3] += 0.05  # Move +5cm in X
            target_pose[2, 3] += 0.02  # Move +2cm in Z
            
            # Call the new function to move the arm
            move_arm_to_pose("right", target_pose)
            
    finally:
        disconnect_arms()