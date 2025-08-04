import argparse
import pybullet as p
import time
import pybullet_data
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('--no-gui', action='store_true', help='Disable GUI mode')
args = parser.parse_args()

LEFT_WHEEL_ID = 0
RIGHT_WHEEL_ID = 1

p.connect(p.DIRECT if args.no_gui else p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF('plane.urdf')


robot = None
velocity = 0.0
frame = 0
reward = 0.0


def reset():
  global robot, velocity, frame, reward
  velocity = 0.0
  frame = 0
  reward = 0

  if robot:
    p.removeBody(robot)

  robot = p.loadURDF(
    'sim/balance/balance.urdf',
    basePosition=[0.0, 0.0, 0.05],
    baseOrientation=p.getQuaternionFromEuler([np.random.uniform(-0.2, 0.2), 0, 0]),
    useFixedBase=False,
  )


reset()

while True:
  keys = p.getKeyboardEvents()

  if ord('o') in keys and keys[ord('o')] & p.KEY_IS_DOWN:
    velocity += 0.5
  if ord('l') in keys and keys[ord('l')] & p.KEY_IS_DOWN:
    velocity -= 0.5

  p.stepSimulation()

  p.setJointMotorControl2(robot, LEFT_WHEEL_ID, p.VELOCITY_CONTROL, targetVelocity=velocity, force=10.0)
  p.setJointMotorControl2(robot, RIGHT_WHEEL_ID, p.VELOCITY_CONTROL, targetVelocity=-velocity, force=10.0)

  _, orientation = p.getBasePositionAndOrientation(robot)
  roll, pitch, yaw = p.getEulerFromQuaternion(orientation)

  if abs(roll) > 0.5 or abs(pitch) > 0.5:
    reset()

  if frame % 10 == 0:
    data = {'frame': frame, 'reward': reward, 'roll': roll, 'pitch': pitch, 'yaw': yaw, 'velocity': velocity}
    print('\n' + '\n'.join(f'{k:<5} = {v: .5f}' for k, v in data.items()) + '\n')

  time.sleep(1.0 / 120.0)
  frame += 1
