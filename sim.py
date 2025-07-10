import pybullet as p
import pybullet_data
import numpy as np
import time

from robots.sim import SimRobot

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")

left_id = p.loadURDF(
    "sim/SO101/so101_new_calib.urdf",
    basePosition=[0.0, 0.08, 0.5],
    baseOrientation=p.getQuaternionFromEuler([-np.pi / 2, 0, 0]),
    useFixedBase=True,
)

right_id = p.loadURDF(
    "sim/SO101/so101_new_calib.urdf",
    basePosition=[0.0, 0.0, 0.5],
    baseOrientation=p.getQuaternionFromEuler([np.pi / 2, 0, 0]),
    useFixedBase=True,
)

num_joints = p.getNumJoints(left_id)
end_effector = "gripper_frame_joint"


def find_joint(name):
    """Finds a joint by its name and returns its info."""
    for i in range(num_joints):
        joint_info = p.getJointInfo(left_id, i)
        joint_name = joint_info[1].decode("utf-8")
        if joint_name == name:
            lower_limit = joint_info[8]
            upper_limit = joint_info[9]
            print(
                f"Found '{name}' joint at index {i}, lower: {lower_limit}, upper: {upper_limit}"
            )
            return i, lower_limit, upper_limit
    print(f"Error: Joint '{name}' not found.")
    return None, None, None


gripper_id, gripper_lower, gripper_upper = find_joint("gripper")

x_slider = p.addUserDebugParameter("Target X", -1.0, 1.0, 1)
y_slider = p.addUserDebugParameter("Target Y", -1.0, 1.0, 1)
z_slider = p.addUserDebugParameter("Target Z", -1.0, 1.0, 1)

roll_slider = p.addUserDebugParameter("Target Roll", -np.pi, np.pi, 0)
pitch_slider = p.addUserDebugParameter("Target Pitch", -np.pi, np.pi, -np.pi / 2)
yaw_slider = p.addUserDebugParameter("Target Yaw", -np.pi, np.pi, 0)


robot = SimRobot()

if __name__ == "__main__":
    while True:
        x = p.readUserDebugParameter(x_slider)
        y = p.readUserDebugParameter(y_slider)
        z = p.readUserDebugParameter(z_slider)

        target_roll = p.readUserDebugParameter(roll_slider)
        target_pitch = p.readUserDebugParameter(pitch_slider)
        target_yaw = p.readUserDebugParameter(yaw_slider)

        arm_joints = [0, 1, 2, 3, 4]

        # LEFT
        target_orientation = p.getQuaternionFromEuler(
            [target_roll, target_pitch, target_yaw]
        )
        pos = robot._get_ik(
            "left",
            [x, y, z],
            target_orientation,
        )
        p.setJointMotorControlArray(
            bodyIndex=left_id,
            jointIndices=arm_joints,
            controlMode=p.POSITION_CONTROL,
            targetPositions=[
                pos.shoulder_pan,
                pos.shoulder_lift,
                pos.elbow_flex,
                pos.wrist_flex,
                pos.wrist_roll,
            ],
            forces=[100.0] * len(arm_joints),
        )

        # RIGHT
        # target_orientation = p.getQuaternionFromEuler(
        #     [target_roll, target_pitch, target_yaw]
        # )
        # pos = robot._get_ik(
        #     "right",
        #     [x, y, z],
        #     target_orientation,
        # )
        # p.setJointMotorControlArray(
        #     bodyIndex=right_id,
        #     jointIndices=arm_joints,
        #     controlMode=p.POSITION_CONTROL,
        #     targetPositions=[
        #         pos.shoulder_pan,
        #         pos.shoulder_lift,
        #         pos.elbow_flex,
        #         pos.wrist_flex,
        #         pos.wrist_roll,
        #     ],
        #     forces=[100.0] * len(arm_joints),
        # )

        gripper_pos = None
        keys = p.getKeyboardEvents()
        for key, state in keys.items():
            if state & p.KEY_WAS_TRIGGERED:
                if key == ord("o"):
                    gripper_pos = gripper_upper
                    print("Gripper: OPEN")
                elif key == ord("c"):
                    gripper_pos = gripper_lower
                    print("Gripper: CLOSE")

        if gripper_pos is not None and gripper_id is not None:
            p.setJointMotorControl2(
                left_id,
                gripper_id,
                p.POSITION_CONTROL,
                targetPosition=gripper_pos,
                force=50.0,
            )

        p.stepSimulation()
        time.sleep(1.0 / 240.0)


p.disconnect()
