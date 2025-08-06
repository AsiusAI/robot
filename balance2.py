import os
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import pybullet_data


GUI = True
GRAVITY = -9.81
MAX_VELOCITY = 20.0
MAX_STEPS = 10_000
FALL_ANGLE_THRESHOLD = 0.5

W_ALIVE = 1.0
W_EFFORT = -0.1
W_TILT = -1.0

LEFT_WHEEL_ID = 0
RIGHT_WHEEL_ID = 1


class BalanceBotEnv(gym.Env):
  def __init__(self):
    super(BalanceBotEnv, self).__init__()

    self.client = p.connect(p.GUI if GUI else p.DIRECT)

    self.action_space = spaces.Box(low=-1, high=1, shape=(1,), dtype=np.float32)

    self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)

    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)

    self.urdf_root_path = os.path.dirname(os.path.abspath(__file__))
    self.robot_urdf_path = os.path.join(self.urdf_root_path, 'sim/balance/balance.urdf')

  def _get_roll(self):
    pos, ori_quat = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
    roll, _, _ = p.getEulerFromQuaternion(ori_quat)
    return roll

  def _get_observation(self):
    roll = self._get_roll()

    _, angular_velocity = p.getBaseVelocity(self.robot, physicsClientId=self.client)
    gyro_x = angular_velocity[0]

    left_wheel_vel = p.getJointState(self.robot, LEFT_WHEEL_ID, physicsClientId=self.client)[1]
    normalized_roll = np.clip(roll / FALL_ANGLE_THRESHOLD, -1.0, 1.0)
    normalized_gyro_x = np.clip(gyro_x / MAX_VELOCITY, -1.0, 1.0)
    normalized_left_wheel_vel = np.clip(left_wheel_vel / MAX_VELOCITY, -1.0, 1.0)

    return np.array([normalized_roll, normalized_gyro_x, normalized_left_wheel_vel], dtype=np.float32)

  def reset(self, seed=None, options=None):
    if seed is not None:
      np.random.seed(seed)

    p.resetSimulation(physicsClientId=self.client)
    p.setGravity(0, 0, GRAVITY, physicsClientId=self.client)
    p.loadURDF('plane.urdf', physicsClientId=self.client)

    start_pos = [0, 0, 0.2]
    start_orientation = p.getQuaternionFromEuler([np.random.uniform(-0.1, 0.1), 0, 0])

    self.robot = p.loadURDF(self.robot_urdf_path, start_pos, start_orientation, useFixedBase=False, physicsClientId=self.client)

    self.step_counter = 0

    observation = self._get_observation()
    return observation, {}

  def step(self, action):
    scaled_action = action * MAX_VELOCITY

    p.setJointMotorControl2(
      self.robot, LEFT_WHEEL_ID, p.VELOCITY_CONTROL, targetVelocity=scaled_action[0], force=10.0, physicsClientId=self.client
    )
    p.setJointMotorControl2(
      self.robot, RIGHT_WHEEL_ID, p.VELOCITY_CONTROL, targetVelocity=-scaled_action[0], force=10.0, physicsClientId=self.client
    )

    p.stepSimulation(physicsClientId=self.client)

    observation = self._get_observation()
    roll = self._get_roll()

    reward = W_ALIVE

    reward += W_EFFORT * np.sum(np.square(action))

    reward += W_TILT * (roll**2)

    self.step_counter += 1

    terminated = False
    if abs(roll) > FALL_ANGLE_THRESHOLD:
      reward = -100
      terminated = True

    truncated = self.step_counter >= MAX_STEPS
    return observation, float(reward), terminated, truncated, {}

  def close(self):
    p.disconnect(self.client)


if __name__ == '__main__':
  from stable_baselines3 import PPO
  from stable_baselines3.common.env_checker import check_env

  env = BalanceBotEnv()
  check_env(env)
  print('Environment check passed!')
  model_path = os.path.join('training', 'saved_models', 'ppo_balancebot_model.zip')

  if os.path.exists(model_path):
    print(f'Loading existing model from {model_path}')
    model = PPO.load(model_path, env=env)
  else:
    print('No existing model found. Creating a new one.')
    model = PPO(
      'MlpPolicy',
      env,
      verbose=1,
      n_steps=2048,
      batch_size=512,
      n_epochs=10,
      gamma=0.99,
      gae_lambda=0.95,
      ent_coef=0.01,
      vf_coef=0.5,
      max_grad_norm=0.5,
    )

  TRAIN_STEPS = 500_000
  try:
    model.learn(total_timesteps=TRAIN_STEPS)
  except KeyboardInterrupt:
    print('Training interrupted. Saving model...')
  finally:
    model.save(model_path)
    print(f'Model saved to {model_path}')
    env.close()
