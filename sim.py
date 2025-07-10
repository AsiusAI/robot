import pybullet as p
import pybullet_data
import numpy as np
import time
import placo
from placo_utils.tf import tf
from scipy.spatial.transform import Rotation

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")
robot_start_pos = [0, 0, 0]
robot_start_orn = p.getQuaternionFromEuler([0, 0, 0])
robot_id = p.loadURDF(
    "sim/SO101/so101_new_calib.urdf",
    robot_start_pos,
    robot_start_orn,
    useFixedBase=True,
)

num_joints = p.getNumJoints(robot_id)
print("Robot joint info:")
for i in range(num_joints):
    print(p.getJointInfo(robot_id, i))

end_effector = "gripper_frame_joint"
end_effector_link_index = 5  # gripper_frame_joint


def find_joint(name):
    """Finds a joint by its name and returns its info."""
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
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

x_slider = p.addUserDebugParameter("Target X", -1.0, 1.0, 0.3)
y_slider = p.addUserDebugParameter("Target Y", -1.0, 1.0, 0)
z_slider = p.addUserDebugParameter("Target Z", 0.0, 1.5, 0.5)

roll_slider = p.addUserDebugParameter("Target Roll", -np.pi, np.pi, 0)
pitch_slider = p.addUserDebugParameter("Target Pitch", -np.pi, np.pi, -np.pi / 2)
yaw_slider = p.addUserDebugParameter("Target Yaw", -np.pi, np.pi, 0)


robot = placo.RobotWrapper(
    "sim/SO101/so101_new_calib.urdf",
)
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)
effector_task = solver.add_frame_task(end_effector, np.eye(4))
effector_task.configure("effector", "soft", 10.0, 1.0)
solver.enable_velocity_limits(True)
solver.dt = 0.01


def calc_inverse_kinematics(position, orientation):
    T_world_frame = np.eye(4)
    rotation = Rotation.from_quat(orientation)
    T_world_frame[:3, :3] = rotation.as_matrix()
    T_world_frame[:3, 3] = position
    effector_task.T_world_frame = T_world_frame
    solver.solve(True)
    robot.update_kinematics()
    return robot.state.q[7:]


if __name__ == "__main__":
    while True:
        target_x = p.readUserDebugParameter(x_slider)
        target_y = p.readUserDebugParameter(y_slider)
        target_z = p.readUserDebugParameter(z_slider)
        target_position = [target_x, target_y, target_z]

        target_roll = p.readUserDebugParameter(roll_slider)
        target_pitch = p.readUserDebugParameter(pitch_slider)
        target_yaw = p.readUserDebugParameter(yaw_slider)

        target_orientation = p.getQuaternionFromEuler(
            [target_roll, target_pitch, target_yaw]
        )

        joint_poses = calc_inverse_kinematics(
            target_position,
            target_orientation,
        )

        arm_joints = [i for i in range(end_effector_link_index + 1)]
        p.setJointMotorControlArray(
            bodyIndex=robot_id,
            jointIndices=arm_joints,
            controlMode=p.POSITION_CONTROL,
            targetPositions=joint_poses[: len(arm_joints)],
            forces=[100.0] * len(arm_joints),
        )

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
                robot_id,
                gripper_id,
                p.POSITION_CONTROL,
                targetPosition=gripper_pos,
                force=50.0,
            )

        p.stepSimulation()
        time.sleep(1.0 / 240.0)


p.disconnect()
