

import gym
import gym_robot2d

env_name = 'robot2d-v0'

env = gym.make(env_name)

obs = env.reset()
print(obs)