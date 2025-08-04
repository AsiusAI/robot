import time
from typing import Tuple

import numpy as np
import pybullet as p
import pybullet_data

from tinygrad import Tensor, TinyJit, nn
from tinygrad.helpers import trange, getenv


GUI = True

EPISODES = getenv('EPISODES', 100)
BATCH_SIZE = 512
REPLAY_BUFFER_SIZE = 10000
LEARNING_RATE = 3e-4
HIDDEN_UNITS = 64
TRAIN_STEPS = 10
DISCOUNT_FACTOR = 0.99
PPO_EPSILON = 0.15
ENTROPY_SCALE = 0.005
VELOCITY_INCREMENT = 1.0
MAX_VELOCITY = 20
MAX_STEPS = 20_000
FALL_ANGLE_THRESHOLD = 0.6


OBSERVATION_SHAPE = (2,)
ACTION_N = 2

INITIAL_ANGLE_MIN_MAX = 0.1
GRAVITY = -9.81

ANGLE_COEF = 1
SPEED_COEF = 0.3
DISTANCE_COEF = 0.3
VELOCITY_COEF = 0.1

TERMINATION_REWARD = -10


class BalanceBotEnv:
  def __init__(self):
    self.client = p.connect(p.GUI if GUI else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
    p.setGravity(0, 0, GRAVITY, physicsClientId=self.client)
    p.loadURDF('plane.urdf', physicsClientId=self.client)
    self.robot = None

  def _get_obs(self) -> np.ndarray:
    _, orientation = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
    roll, _, _ = p.getEulerFromQuaternion(orientation)
    _, angular_vel = p.getBaseVelocity(self.robot, physicsClientId=self.client)
    roll_velocity = angular_vel[0]
    return np.array([roll, roll_velocity], dtype=np.float32)

  def reset(self) -> Tuple[np.ndarray, dict]:
    start_orientation = p.getQuaternionFromEuler([np.random.uniform(-INITIAL_ANGLE_MIN_MAX, INITIAL_ANGLE_MIN_MAX), 0, 0])
    if self.robot is not None:
      p.removeBody(self.robot)
    self.robot = p.loadURDF(
      'sim/balance/balance.urdf',
      basePosition=[0, 0, 0],
      baseOrientation=start_orientation,
      useFixedBase=False,
      physicsClientId=self.client,
    )
    self.velocity = 0.0
    self.steps = 0
    return self._get_obs(), {}

  def step(self, action: int) -> Tuple[np.ndarray, float, bool, bool, dict]:
    if action == 0:
      self.velocity += VELOCITY_INCREMENT
    elif action == 1:
      self.velocity -= VELOCITY_INCREMENT
    self.velocity = np.clip(self.velocity, -MAX_VELOCITY, MAX_VELOCITY)

    p.setJointMotorControl2(self.robot, 0, p.VELOCITY_CONTROL, targetVelocity=self.velocity, force=10.0, physicsClientId=self.client)
    p.setJointMotorControl2(self.robot, 1, p.VELOCITY_CONTROL, targetVelocity=-self.velocity, force=10.0, physicsClientId=self.client)
    p.stepSimulation(physicsClientId=self.client)

    self.steps += 1
    obs = self._get_obs()
    pos, _ = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
    linear_vel, _ = p.getBaseVelocity(self.robot, physicsClientId=self.client)

    roll = obs[0]
    terminated = abs(roll) > FALL_ANGLE_THRESHOLD

    if not terminated:
      angle_penalty = abs(roll) / FALL_ANGLE_THRESHOLD
      speed_penalty = (linear_vel[0] ** 2 + linear_vel[1] ** 2) ** 0.5
      distance_penalty = (pos[0] ** 2 + pos[1] ** 2) ** 0.5
      motor_velocity_penalty = abs(self.velocity) / MAX_VELOCITY

      reward = (
        1
        - ANGLE_COEF * angle_penalty
        - SPEED_COEF * speed_penalty
        - DISTANCE_COEF * distance_penalty
        - VELOCITY_COEF * motor_velocity_penalty
      )
    else:
      reward = TERMINATION_REWARD

    truncated = self.steps >= MAX_STEPS
    return obs, reward, terminated, truncated, {}

  def close(self):
    p.disconnect(self.client)


class ActorCritic:
  def __init__(self, in_features, out_features, hidden_state=HIDDEN_UNITS):
    self.l1 = nn.Linear(in_features, hidden_state)
    self.l2 = nn.Linear(hidden_state, out_features)
    self.c1 = nn.Linear(in_features, hidden_state)
    self.c2 = nn.Linear(hidden_state, 1)

  def __call__(self, obs: Tensor) -> Tuple[Tensor, Tensor]:
    x = self.l1(obs).tanh()
    act = self.l2(x).log_softmax()
    x = self.c1(obs).relu()
    return act, self.c2(x)


if __name__ == '__main__':
  env = BalanceBotEnv()
  model = ActorCritic(OBSERVATION_SHAPE[0], ACTION_N)
  opt = nn.optim.Adam(nn.state.get_parameters(model), lr=LEARNING_RATE)

  @TinyJit
  def train_step(x: Tensor, selected_action: Tensor, reward: Tensor, old_log_dist: Tensor) -> Tuple[Tensor, Tensor, Tensor]:
    with Tensor.train():
      log_dist, value = model(x)
      action_mask = (
        selected_action.reshape(-1, 1) == Tensor.arange(log_dist.shape[1]).reshape(1, -1).expand(selected_action.shape[0], -1)
      ).float()
      advantage = reward.reshape(-1, 1) - value
      masked_advantage = action_mask * advantage.detach()
      ratios = (log_dist - old_log_dist).exp()
      unclipped_ratio = masked_advantage * ratios
      clipped_ratio = masked_advantage * ratios.clip(1 - PPO_EPSILON, 1 + PPO_EPSILON)
      action_loss = -unclipped_ratio.minimum(clipped_ratio).sum(-1).mean()
      entropy_loss = (log_dist.exp() * log_dist).sum(-1).mean()
      critic_loss = advantage.square().mean()
      opt.zero_grad()
      (action_loss + entropy_loss * ENTROPY_SCALE + critic_loss).backward()
      opt.step()
      return action_loss.realize(), entropy_loss.realize(), critic_loss.realize()

  @TinyJit
  def get_action(obs: Tensor) -> Tensor:
    return (model(obs)[0] - Tensor.rand(*model(obs)[0].shape).log().neg().log().neg()).argmax().realize()

  st, steps = time.perf_counter(), 0
  Xn, An, Rn = [], [], []
  total_rewards_log = []
  for episode_number in (t := trange(EPISODES)):
    get_action.reset()
    obs: np.ndarray = env.reset()[0]
    rews, terminated, truncated = [], False, False
    while not terminated and not truncated:
      act = get_action(Tensor(obs)).item()
      Xn.append(np.copy(obs))
      An.append(act)
      obs, rew, terminated, truncated, _ = env.step(act)
      rews.append(float(rew))
    steps += len(rews)
    total_rewards_log.append(sum(rews))

    discounts = np.power(DISCOUNT_FACTOR, np.arange(len(rews)))
    Rn += [np.sum(rews[i:] * discounts[: len(rews) - i]) for i in range(len(rews))]
    Xn, An, Rn = Xn[-REPLAY_BUFFER_SIZE:], An[-REPLAY_BUFFER_SIZE:], Rn[-REPLAY_BUFFER_SIZE:]
    if len(Xn) < BATCH_SIZE:
      continue  # Don't train until we have enough data

    X, A, R = Tensor(Xn), Tensor(An), Tensor(Rn)

    old_log_dist = model(X)[0].detach()
    for i in range(TRAIN_STEPS):
      samples = Tensor.randint(BATCH_SIZE, high=X.shape[0]).realize()
      action_loss, entropy_loss, critic_loss = train_step(X[samples], A[samples], R[samples], old_log_dist[samples])
    avg_rew = sum(total_rewards_log[-20:]) / len(total_rewards_log[-20:])
    t.set_description(f'sz:{len(Xn):5d} avg_rew(20):{avg_rew:8.2f} last_rew:{sum(rews):8.2f}')

  env.close()
