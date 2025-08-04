import argparse
import pybullet as p
import pybullet_data
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('--direct', action='store_true', help='Disable GUI mode')
args = parser.parse_args()

LEFT_WHEEL_ID = 0
RIGHT_WHEEL_ID = 1
MAX_STEPS = 500

p.connect(p.DIRECT if args.direct else p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


robot = None
velocity = 0.0
steps = 0
reward = 0.0
times = 0


def reset():
  global robot, velocity, steps, reward, times
  velocity = 0.0
  steps = 0
  reward = 0
  times += 1

  p.resetSimulation()
  p.setGravity(0, 0, -9.81)
  p.loadURDF('plane.urdf')

  robot = p.loadURDF(
    'sim/balance/balance.urdf',
    basePosition=[0.0, 0.0, 0.05],
    baseOrientation=p.getQuaternionFromEuler([np.random.uniform(-0.2, 0.2), 0, 0]),
    useFixedBase=False,
  )


reset()

while True:
  p.stepSimulation()

  p.setJointMotorControl2(robot, LEFT_WHEEL_ID, p.VELOCITY_CONTROL, targetVelocity=velocity, force=10.0)
  p.setJointMotorControl2(robot, RIGHT_WHEEL_ID, p.VELOCITY_CONTROL, targetVelocity=-velocity, force=10.0)

  if steps >= MAX_STEPS:
    reset()

  _, orientation = p.getBasePositionAndOrientation(robot)
  roll, pitch, yaw = p.getEulerFromQuaternion(orientation)

  if abs(roll) > 0.5 or abs(pitch) > 0.5:
    reset()

  if steps % 100 == 0:
    data = {
      'times': times,
      'frame': steps,
      'reward': reward,
      'roll': roll,
      'pitch': pitch,
      'yaw': yaw,
      'velocity': velocity,
    }
    print('\n' + '\n'.join(f'{k:<10} = {v: .5f}' for k, v in data.items()) + '\n')

  steps += 1
