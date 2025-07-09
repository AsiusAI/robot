import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation
import time

# Start PyBullet
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

# Load your robot
robot_id = p.loadURDF("sim/SO101/so101_new_calib.urdf")

# Get joint info
joint_names = []
joint_indices = []
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    joint_names.append(info[1].decode('utf-8'))
    joint_indices.append(i)
    print(f"Joint {i}: {info[1].decode('utf-8')}")

# Find your arm joints
ARM_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
arm_joint_indices = [joint_names.index(name) for name in ARM_JOINT_NAMES if name in joint_names]

def test_ik_consistency():
    """Test if same target pose gives same joint angles"""
    # Fixed test pose
    target_pos = [0.4, 0.0, 0.3]
    target_quat = [0, 0, 0, 1]  # Identity quaternion
    
    results = []
    for i in range(5):
        # Reset to neutral pose
        neutral_joints = [0, -45, 90, 0, 0]  # degrees
        for j, joint_idx in enumerate(arm_joint_indices):
            p.resetJointState(robot_id, joint_idx, np.deg2rad(neutral_joints[j]))
        
        # Use PyBullet's IK
        joint_poses = p.calculateInverseKinematics(
            robot_id, 
            p.getNumJoints(robot_id)-1,  # End effector link
            target_pos, 
            target_quat
        )
        
        # Extract arm joint angles
        arm_angles = [np.rad2deg(joint_poses[idx]) for idx in arm_joint_indices]
        results.append(arm_angles)
        print(f"Test {i+1}: {arm_angles}")
    
    # Check consistency
    for i in range(1, len(results)):
        diff = np.abs(np.array(results[i]) - np.array(results[0]))
        print(f"Difference from first: {diff}")
        if np.max(diff) > 1.0:  # 1 degree tolerance
            print("WARNING: IK solutions are inconsistent!")
        
def visualize_pose(pos, quat):
    """Visualize target pose and move robot"""
    # Convert to joint angles
    joint_poses = p.calculateInverseKinematics(robot_id, p.getNumJoints(robot_id)-1, pos, quat)
    
    # Set joint positions
    for i, joint_idx in enumerate(arm_joint_indices):
        p.setJointMotorControl2(robot_id, joint_idx, p.POSITION_CONTROL, joint_poses[joint_idx])
    
    # Add visual marker at target
    p.addUserDebugLine([0,0,0], pos, [1,0,0], 3, 1.0)
    
    # Step simulation
    for _ in range(240):  # 4 seconds at 60fps
        p.stepSimulation()
        time.sleep(1./60.)

def test_vr_mapping():
    """Test VR coordinate mapping"""
    VR_WORKSPACE = {
        "x": (0.2, 0.6),
        "y": (-0.3, 0.3),  
        "z": (0.1, 0.5),
    }
    
    def vr_to_robot_coords(vr_pos):
        return [
            np.interp(vr_pos[0], [-1, 1], VR_WORKSPACE["x"]),
            np.interp(vr_pos[1], [-1, 1], VR_WORKSPACE["y"]),
            np.interp(vr_pos[2], [-1, 1], VR_WORKSPACE["z"])
        ]
    
    # Test VR positions
    test_vr_positions = [
        [0.5, 0.5, 0.5],
        [0.0, 0.0, 0.0],
        [-0.5, -0.5, -0.5]
    ]
    
    for vr_pos in test_vr_positions:
        robot_pos = vr_to_robot_coords(vr_pos)
        print(f"VR {vr_pos} -> Robot {robot_pos}")
        visualize_pose(robot_pos, [0, 0, 0, 1])

if __name__ == "__main__":
    print("Testing IK consistency...")
    test_ik_consistency()
    
    print("\nTesting VR mapping...")
    test_vr_mapping()
    
    # Keep simulation running
    while True:
        p.stepSimulation()
        time.sleep(1./60.)