from gym.envs.registration import register

register(
	id = 'robot2d-v0', 
	entry_point = 'gym_robot2d.envs:Robot2dEnv'
)
