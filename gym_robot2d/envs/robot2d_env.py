# Call Library robot
import sys
#a = sys.path.insert(1,'../../../robot_2d')
#print(sys.path)

from robot2d import Robot2D

# Core libraries
import gym 
import numpy as np 
import math
import time
import random
import os

# Utils for GYM 
from gym import spaces, error, utils
from gym.utils import seeding





class Robot2dEnv(gym.Env): 
	def __init__(self):
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

		super(Robot2dEnv, self).__init__()

		# Initialize variables
		self.state = None
		self.viewer = None 
		self.robot = Robot2D(dT = 0.05, is_render=True, is_goal=True)
		self.eps_err = 0.4
		self.steps = 0
		seeding

		self.max_action_magnitude = 5

		# Position variables
		self.x_goal = 0
		self.y_goal = 0 
		self.robot_goal = np.array([self.x_goal, self.y_goal])

		# Spaces Environment
		# maybe remove below
		self.action_space = spaces.Box(low = -np.finfo(np.float32).max, 
			high = np.finfo(np.float32).max, shape = (2,1), dtype= np.float32)


	def step(self,action):

		# State Update 
		vx = action[0]
		vy = action[1]
		# Clip actions
		vx = np.clip(vx, -self.max_action_magnitude, self.max_action_magnitude)
		vy = np.clip(vy, -self.max_action_magnitude, self.max_action_magnitude)


		self.robot.step(vx,vy)
		robot_pos = np.array([self.robot.xr, self.robot.yr])
		is_crashed = self.robot.is_crashed()

		# Observation Update
		self.robot.scanning()
		xls_r = self.robot.xls - self.robot.xr
		yls_r = self.robot.yls - self.robot.yr
		pairs = list(zip(xls_r, yls_r))
		random.shuffle(pairs)

		obs = np.array([val for pair in pairs for val in pair])

		# Done condition
		done = bool(
			is_crashed 
			or np.linalg.norm(self.robot_goal-robot_pos) <= self.eps_err
			or self.steps > 300
			)

		if not done:
			self.steps += 1
			reward = - 0.1* np.linalg.norm(self.robot_goal-robot_pos)

		else:
			if is_crashed: 
				reward = -10
			elif (np.linalg.norm(self.robot_goal-robot_pos) <= self.eps_err):
				reward = 10
			else: 
				reward = 0
			# Acabó el número de episodios

		return np.concatenate( (robot_pos - self.robot_goal,obs)) , reward, done, {}

	def reset(self): # Return to initial state
		self.robot.reset()
		
		self.steps = 0

		robot_pos = np.array([self.robot.xr, self.robot.yr])
		self.robot_goal = np.array([self.robot.xg, self.robot.yg])

		# First observation
		self.robot.scanning()
		obs = np.array(self.robot.xls +  self.robot.yls)
		#print("The environment has been reset")
		return np.concatenate( (robot_pos - self.robot_goal,obs))


	def render(self):
		self.robot.render()

	def close(self):
		self.robot.close()