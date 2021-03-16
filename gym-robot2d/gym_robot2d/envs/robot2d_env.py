# Call Library robot
import sys
sys.path.insert(1,'../../../2d-robot')
from robot import *



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
	def __init__(self, x_goal=4, y_goal=4):
		"""
		Description: 
			A 2D Robot is placed on an empty environment
			sorrounded by squared and round obstacles (see Simulation). 
			The robot starts at the center making its way throught the 
			obstacles to reachs its goal position. 

		Observation: 
			Type: Box (5)
			Num 	Observation 	Min 	Max
			0  		X Position		-5		5
			1 		X Velocity		-5		5

		Action: 
			Type: Box (2)
			Num 	Action 	 Min 	Max
			0	 	Vx	 -Inf	Inf
			1 		Vy    -Inf 	Inf
		"""

		super(Robot2d, self).__init__()

		# Initialize variables
		self.state = None
		self.viewer = None 
		self.robot = Robot(dT = 0.1)
		self.eps_err = 0.05
		self.steps = 0

		# Position variables
		self.x_goal = x_goal 
		self.y_goal = y_goal 
		self.robot_goal = np.array([self.x_goal, self.y_goal])

		# Spaces Environment
		self.observation_space = spaces.Box(low = self.robot.env_min_size, 
			high = self.robot.env_max_size, shape = (2,1), dtype=np.float32)
		self.action_space = spaces.Box(low = -np.finfo(np.float32).max, 
			high = np.finfo(np.float32).max, shape = (2,1), dtype= np.float32)

	def step(self,action):

		# State Update 
		vx = action[0]
		vy = action[1]
		self.robot.step(vx,vy)
		robot_pos = np.array([self.robot.xr, self.robot.yr])
		is_crashed = self.robot.is_crashed()

		# Observation Update
		self.robot.scanning()
		obs = np.array(self.robot.xls +  self.robot.yls)


		# Done condition
		done = bool(
			is_crashed 
			or np.linalg.norm(self.robot_goal-robot_pos) <= self.eps_err
			or self.steps == 200
			)

		if not done:
			self.steps += 1
			reward = - np.linalg.norm(self.robot_goal-robot_pos)

		else:
			if is_crashed: 
				reward = -100
			else: 
				reward = 0
			# Acabó el número de episodiroos

		return np.concatenate( (robot_pos - self.robot_goal,obs)) , reward, done, {}

	def reset(self): # Return to initial state
		self.robot.reset()
		robot_pos = np.array([self.robot.xr, self.robot.yr])
		# Observation Update
		self.robot.scanning()
		obs = np.array(self.robot.xls +  self.robot.yls)
		print("The environment has been reset")
		return np.concatenate( (robot_pos - self.robot_goal,obs))


	def render(self):
		self.robot.render()
