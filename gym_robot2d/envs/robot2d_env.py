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
	def __init__(self, 
				dT = 0.05,
				is_goal = True,
				is_rotated = True,
				eps_err = 0.4,
				max_action_magnitude = 5
				):
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
		self.is_rotated = is_rotated
		self.is_goal = is_goal
		self.dT = dT

		self.robot = Robot2D(dT = self.dT, is_render=True, is_goal = self.is_goal, is_rotated = self.is_rotated)
		self.eps_err = eps_err
		self.steps = 0

		self.max_action_magnitude = max_action_magnitude

		# Position variables
		self.robot_goal = np.array([0., 0, 0.]) if self.is_rotated else np.array([0., 0.])

		# Spaces Environment
		# maybe remove below
		self.action_space = \
			spaces.Box(low = -max_action_magnitude, 
			high = max_action_magnitude, shape = (3,1), dtype= np.float32) if self.is_rotated else \
			spaces.Box(low = -max_action_magnitude, 
			high = max_action_magnitude, shape = (2,1), dtype= np.float32) 


	def step(self,action):
		if self.is_rotated:
			required_dim = 3
		else:
			required_dim = 2

		if np.shape(action)[0] != required_dim:
			raise Exception("Wrong action dim. Expected to have {} but got {} instead.".format(np.shape(action)[0], required_dim))

		# State Update 
		vx = action[0]
		vy = action[1]
		w = action[2] if self.is_rotated else 0.

		# Clip actions
		vx = np.clip(vx, -self.max_action_magnitude, self.max_action_magnitude)
		vy = np.clip(vy, -self.max_action_magnitude, self.max_action_magnitude)
		w = np.clip(w, -self.max_action_magnitude, self.max_action_magnitude)

		self.robot.step(vx,vy,w)

		if self.is_rotated:
			robot_pos = np.array([self.robot.xr, self.robot.yr, self.robot.thr])
		else:
			robot_pos = np.array([self.robot.xr, self.robot.yr])

		is_crashed = self.robot.is_crashed()

		# Observation Update
		touches = self.robot.scanning()
		xls_r = self.robot.xls - self.robot.xr
		yls_r = self.robot.yls - self.robot.yr
		xls_r = xls_r[touches]
		yls_r = yls_r[touches]
		
		if xls_r.size == 0:
			xls_r = np.array([self.robot.max_range])
			yls_r = np.array([0.])

		# Random shuffle
		#pairs = list(zip(xls_r, yls_r))
		#random.shuffle(pairs)
		#obs = np.array([val for pair in pairs for val in pair])
		
		obs = np.concatenate( (xls_r, yls_r) )

		# Check goal in front of robot to be successful
		xg = self.robot.xg
		yg = self.robot.yg

		th = self.robot.thr
		xr = self.robot.xr
		yr = self.robot.yr
		fov = np.pi/3
		lt = 1.2

		th_p = th + fov / 2
		th_m = th - fov / 2

		y_th_p = np.tan(th_p) * ( xg - xr) + yr
		y_th_m = np.tan(th_m) * ( xg - xr) + yr

		is_success = False

		if  (yg > y_th_m) and \
			(yg < y_th_p) and \
			( np.linalg.norm( [ yg - yr, xg - xr] ) < lt ):
			is_success = True
		
		# Rewards 
		r_collide = -10
		r_success = 10
		r_alive = 0.0
		r_nav = r_alive 

		# Done condition
		done = bool(
			is_crashed 
			or is_success #np.linalg.norm(self.robot_goal[0:2] -robot_pos[0:2]) <= self.eps_err
			or self.steps > 200
			)

		if not done:
			self.steps += 1
			# Dense reward navigation
			# reward = - 0.1* np.linalg.norm(self.robot_goal[0:2]-robot_pos[0:2])
			# Sparse reward exploration
			reward = r_nav
		else:
			if is_crashed: 
				reward = r_collide
			elif is_success:
				reward = r_success
			else: 
				reward = r_nav
			# Acabó el número de episodios

		return np.concatenate( (robot_pos ,obs)) , reward, done, {} #- self.robot_goal

	def reset(self): # Return to initial state
		self.robot.reset()
		
		self.steps = 0

		if self.is_rotated:
			robot_pos = np.array([self.robot.xr, self.robot.yr, self.robot.thr])
		else:
			robot_pos = np.array([self.robot.xr, self.robot.yr])

		self.robot_goal = 	np.array([self.robot.xg, self.robot.yg, self.robot.thg]) \
							if self.is_rotated else  \
							np.array([self.robot.xg, self.robot.yg])

		# First observation
		touches = self.robot.scanning()
		xls_r = self.robot.xls - self.robot.xr
		yls_r = self.robot.yls - self.robot.yr

		xls_r = xls_r[touches]
		yls_r = yls_r[touches]
		
		if xls_r.size == 0:
			xls_r = np.array([self.robot.max_range])
			yls_r = np.array([0.])

		#obs = np.array(self.robot.xls +  self.robot.yls)
		obs = np.concatenate( (xls_r, yls_r) )
		#print("The environment has been reset")
		return np.concatenate( (robot_pos ,obs)) # - self.robot_goal


	def render(self):
		self.robot.render()

	def close(self):
		self.robot.close()