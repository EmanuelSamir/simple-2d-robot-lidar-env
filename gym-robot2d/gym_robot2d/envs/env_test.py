

import gym
from robot2d_env import *
import time

env_name = 'robot2d-v0'

#env = gym.make(env_name)

env = Robot2d()

obs = env.reset()
print(obs)

N = 100
vx = 0.5
vy = 0.5

for _ in range(N):
    obs, r, done, _ = env.step([ vy, vy ]  )
    print('reward: {}, it is done: {}'.format(r, done))
    env.render()
    time.sleep(0.1)