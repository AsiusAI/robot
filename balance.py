import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

body = p.loadURDF(
    "sim/balance/balance.urdf",
    basePosition=[0.0, 0.0, 0.0],
    useFixedBase=False,
)

frame_counter = 0
while True:
    print("step")
    p.stepSimulation()
    time.sleep(1.0 / 120.0)
    frame_counter += 1
