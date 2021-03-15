# Core libraries
import gym 
import numpy as np 
import math
import time
import os

# ROS messages for communicatio

# Utils for GYM 
from gym import spaces, error, utils
from gym.utils import seeding


THRESHOLD_MAP_X  = 5 
THRESHOLD_MAP_Y = 5 


class Robot2d(gym.Env): 
	def __init__(self):
		"""
		Description: 
			A 2D Robot is placed on an empty environment
			sorrounded by squared and round obstacles (see Simulation). 
			The robot starts at the center making its way throught the 
			obstacles to reachs its goal position. 

		Observation: 
			Type: Box (5)
			Num 	Observation 	Min 		Max
			0  		X Position		-5			5
			1 		X Velocity		-Inf		Inf
			2 		Y position 		-5 			5
			3		Y Velocity 		-Inf		Inf
			4		LiDAR Range 	min_d[90]	max_d[90]  // Evaluation

		Action: 
			Type: Box (2)
			Num 	Action 	 Min 	Max
			0	 	Acc_x	 -Inf	Inf
			1 		Acc_y    -Inf 	Inf
		"""

		super(Robot2d, self).__init__()

		# Initialize variables
		self.dt - 0.01 # Time delta

		self.state = None
		self.viewer = None 

		self.x_goal = 3.2 # Example
		self.y_goal = 3.2 # Example
		self.x_dot_goal = 0
		self.y_dot_goal = 0

		self.err_eps = 0.05

		# Observation Space
		low = np.array([-THRESHOLD_MAP_X, 
						-np.finfo(np.float32).max,
						-THRESHOLD_MAP_Y,
						-np.finfo(np.float32).max],dtype=np.float32) # Add min box 00 points of lidar

		high = np.array([THRESHOLD_MAP_X, 
						np.finfo(np.float32).max,
						THRESHOLD_MAP_Y,
						np.finfo(np.float32).max],dtype=np.float32) # Add máx 90 points of lidar

		self.observation_space = spaces.Box(low, high, dtype=np
			.float32)

		self.action_space = spaces.Box(low = -np.finfo(np.float32).max, high = np.finfo(np.float32).max, shape = (2,1), dtype= np.float32)


	def step(self,action): 
		acc_x = action[0]
		acc_y = action[1]

		# Robot state update
		x, x_dot, y, y_dot = self.state 
		x = x + self.dt*x_dot
		x_dot = x_dot + self.dt*acc_x
		y = y + self.dt*y_dot
		y_dot = y_dot + self.dt*acc_y

		self.state = (x, x_dot, y, y_dot)

		err_radius = np.sqrt( (self.x_goal - x)**2 + (self.y - y)**2  )

		# Set done condition based on the desired goal: The robot stops near to the goal position 
		done = bool(
			err_radius <= self.err_eps and
			x_dot == self.x_dot_goal and
			y_dot == self.y_dot_goal)

		# Reward
		return np.array(self.state), reward, done, {}

	def reset(self): # Return to initial state

	def close(self):