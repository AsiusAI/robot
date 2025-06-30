import pybullet as p
import pybullet_data
import time
import os

physicsClient = p.connect(p.GUI)

script_path = os.path.dirname(os.path.abspath(__file__))

robot_path = os.path.join(script_path, "urdf", "robot", "urdf")

p.setAdditionalSearchPath(robot_path)

p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")

startPos = [0, 0, 0.1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

urdf_file = os.path.join(robot_path, "robot.urdf") 

print(f"Loading URDF from: {urdf_file}")

try:
    robotId = p.loadURDF(urdf_file, startPos, startOrientation)
    print("Robot loaded successfully!")
    
    # Set up materials for better visualization
    silver_color = [0.7, 0.7, 0.7, 1.0]  # Silver color with full alpha
    
    # Apply materials to all links
    num_joints = p.getNumJoints(robotId)
    print(f"Total number of joints: {num_joints}")
    
    for i in range(-1, num_joints):  # -1 for base_link
        link_id = i if i >= 0 else -1
        p.changeVisualShape(robotId, link_id, rgbaColor=silver_color)
    
    # Debug: Print all joint information
    print("\nAll joints:")
    for i in range(num_joints):
        joint_info = p.getJointInfo(robotId, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        print(f"Joint {i}: {joint_name} (type: {joint_type})")
    
    # Get joint indices for different types of joints
    wheel_joints = []  # Drive wheels (left and right)
    caster_joints = []  # Caster wheel and fork joints
    
    for i in range(num_joints):
        joint_info = p.getJointInfo(robotId, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        
        # Look for revolute joints (type 0) that might be wheels
        if joint_type == 0:  # REVOLUTE joint
            # Check if this is a caster joint (Revolute 1 or 2)
            if "Revolute 1" in joint_name or "Revolute 2" in joint_name:
                caster_joints.append(i)
                print(f"Found caster joint: {joint_name} at index {i}")
            else:
                wheel_joints.append(i)
                print(f"Found drive wheel joint: {joint_name} at index {i}")
    
    # Enable motor control for drive wheels
    for joint_idx in wheel_joints:
        # Disable the default motor control first
        p.setJointMotorControl2(robotId, joint_idx, p.VELOCITY_CONTROL, force=0)
        # Then enable our custom control
        p.setJointMotorControl2(robotId, joint_idx, p.VELOCITY_CONTROL, force=20)
    
    # Configure caster joints for free rotation (no motor control)
    for joint_idx in caster_joints:
        # Disable motor control completely for free rotation
        p.setJointMotorControl2(robotId, joint_idx, p.VELOCITY_CONTROL, force=0)
        # Add some damping to prevent excessive oscillation
        p.changeDynamics(robotId, joint_idx, lateralFriction=0.1, spinningFriction=0.1)
    
    print(f"Found {len(wheel_joints)} drive wheel joints")
    print(f"Found {len(caster_joints)} caster joints")
    
except p.error as e:
    print("Error loading URDF!")
    print(e)

if 'robotId' in locals():
    try:
        # Control variables
        left_wheel_velocity = 0
        right_wheel_velocity = 0
        
        print("\nControls (click on GUI window first, then use keys):")
        print("W/S - Forward/Backward")
        print("A/D - Turn Left/Right")
        print("Q/E - Stop")
        print("ESC - Exit")
        print("\nCaster wheel and fork will rotate freely!")
        
        # Disable PyBullet's default keyboard shortcuts
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
        
        for i in range(10000):
            # Handle keyboard input - this captures from the GUI window
            keys = p.getKeyboardEvents()
            for key, state in keys.items():
                if state & p.KEY_WAS_TRIGGERED:
                    if key == ord('w'):  # Forward
                        left_wheel_velocity = 10
                        right_wheel_velocity = -10
                        print("Moving forward")
                    elif key == ord('s'):  # Backward
                        left_wheel_velocity = -10
                        right_wheel_velocity = 10
                        print("Moving backward")
                    elif key == ord('a'):  # Turn left
                        left_wheel_velocity = -10
                        right_wheel_velocity = -10
                        print("Turning left")
                    elif key == ord('d'):  # Turn right
                        left_wheel_velocity = 10
                        right_wheel_velocity = 10
                        print("Turning right")
                    elif key == ord('q') or key == ord('e'):  # Stop
                        left_wheel_velocity = 0
                        right_wheel_velocity = 0
                        print("Stopping")
                    elif key == ord('esc'):  # Exit
                        print("Exiting...")
                        raise KeyboardInterrupt
            
            # Apply wheel velocities to drive wheels only
            if len(wheel_joints) >= 2:
                p.setJointMotorControl2(robotId, wheel_joints[0], p.VELOCITY_CONTROL, targetVelocity=left_wheel_velocity, force=20)
                p.setJointMotorControl2(robotId, wheel_joints[1], p.VELOCITY_CONTROL, targetVelocity=right_wheel_velocity, force=20)
                
                # Debug: Print current joint states every second
                if i % 240 == 0:  # Print every second
                    print("\n--- Joint States ---")
                    for j, joint_idx in enumerate(wheel_joints):
                        joint_state = p.getJointState(robotId, joint_idx)
                        joint_info = p.getJointInfo(robotId, joint_idx)
                        joint_name = joint_info[1].decode('utf-8')
                        print(f"Drive wheel {joint_name}: velocity={joint_state[1]:.2f}, position={joint_state[0]:.2f}")
                    
                    for j, joint_idx in enumerate(caster_joints):
                        joint_state = p.getJointState(robotId, joint_idx)
                        joint_info = p.getJointInfo(robotId, joint_idx)
                        joint_name = joint_info[1].decode('utf-8')
                        print(f"Caster {joint_name}: velocity={joint_state[1]:.2f}, position={joint_state[0]:.2f}")
            
            p.stepSimulation()
            time.sleep(1./240.)
            
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()
else:
    print("Could not load robot, exiting.")
    p.disconnect()