# Call Library robot
import sys
sys.path.insert(1,'/home/rauloestu/2d-simple-robot-lidar-env/2d-robot')
import robot

# Core libraries
import gym 
import numpy as np 
import math
import time
import os

# Utils for GYM 
from gym import spaces, error, utils
from gym.utils import seeding




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
		Action: 
			Type: Box (2)
			Num 	Action 	 Min 	Max
			0	 	Acc_x	 -Inf	Inf
			1 		Acc_y    -Inf 	Inf
		"""

		super(Robot2d, self).__init__()

		# Initialize variables
		self.state = None
		self.viewer = None 
		self.robot = Robot()

		# Spaces Environment
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

		# State Update 
		vx = action[0]
		vy = action[1]
		state_robot, warn =self.robot.step(vx,vy)
		flag_crash = self.robot.is_crashed()

		# Observation Update
		x_laser_s, y_lases_s = self.robot.scanning()

		# Done condition

		reward = self.reward_evaluation()# Reward evaluation - To complete


		return np.array(self.state), reward, done, {}

	def reset(self): # Return to initial state

	def close(self):