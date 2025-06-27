import pybullet as p
import pybullet_data
import time
import os

# --- Basic PyBullet Setup ---
# Connect to the physics server
physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version

# Add the pybullet_data path for access to default objects
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity and load a ground plane
p.setGravity(0, 0, -9.81)   
planeId = p.loadURDF("plane.urdf")

# --- Load Your Robot ---
# Define the starting position and orientation of your robot
startPos = [0, 0, 0.1] # x, y, z position
startOrientation = p.getQuaternionFromEuler([0, 0, 0]) # x, y, z orientation

# Construct the full path to your URDF file
# Make sure the 'your_robot_urdf_folder' is the one you exported from Fusion
urdf_file_path = os.path.join("your_robot_urdf_folder", "your_robot_name.urdf")

robotId = p.loadURDF(urdf_file_path, startPos, startOrientation)

# --- Simulation Loop ---
try:
    # Run the simulation for a long time
    for i in range(10000):
        p.stepSimulation()
        time.sleep(1./240.) # Sleep to match the simulation step time
except KeyboardInterrupt:
    pass
finally:
    p.disconnect()