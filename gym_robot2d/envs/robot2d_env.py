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
from collections import deque, namedtuple


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

		self.queue_state = deque(maxlen = 10)
		self.queue_nomotion = deque(maxlen = 50)

		self.final_state = 0
		self.max_action_magnitude = max_action_magnitude

		# Position variables
		self.robot_goal = np.array([0., 0, 0.]) if self.is_rotated else np.array([0., 0.])

		self.xr0 = 0
		self.yr0 = 0
		self.thr0 = 0

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


		# Transform robot pose in t wrt robot initial frame
		I_T_r = pose2mat_2d(self.robot.xr, self.robot.yr, self.robot.thr)
		I_T_r0 = pose2mat_2d(self.xr0, self.yr0, self.thr0)
		r0_T_I = inverse_mat_2d(I_T_r0)
		r0_T_r = r0_T_I.dot(I_T_r)
		xr_r0, yr_r0, thr_r0 = mat2pose_2d(r0_T_r)		

		if self.is_rotated:
			robot_pos = np.array([xr_r0, yr_r0, thr_r0])
		else:
			robot_pos = np.array([xr_r0, yr_r0])

		r_T_I = inverse_mat_2d(I_T_r)

		# Check if robot is crashed

		is_crashed = self.robot.is_crashed()

		# Observation Update

		## Transform lidar to robot inertia frame
		touches = self.robot.scanning()
		
		xls = self.robot.xls[touches]
		yls = self.robot.yls[touches]

		if xls.size == 0:
			xls = np.array([self.robot.max_range])
			yls = np.array([0.])

		xls_r = []
		yls_r = []

		for xl, yl in zip(xls, yls):
			I_T_l = pose2mat_2d(xl, yl, 0)
			r_T_l = r_T_I.dot(I_T_l)
			xl_r, yl_r, _ = mat2pose_2d(r_T_l)
			xls_r.append(xl_r)
			yls_r.append(yl_r)


		xls_r = np.array(xls_r)
		yls_r = np.array(yls_r)
		
		obs = np.concatenate( (xls_r, yls_r) )

		# Check goal in front of robot to be successful

		## Transform goal position to robot inertia frame
		I_T_g = pose2mat_2d(self.robot.xg, self.robot.yg, self.robot.thg)
		r_T_g = r_T_I.dot(I_T_g)
		xg_r, yg_r, _ = mat2pose_2d(r_T_g)

		## Notice we do not use from Transformation because goal does not have yet orientation
		thg_r = np.arctan2(yg_r, xg_r)

		self.robot_goal = 	np.array([xg_r, yg_r, thg_r]) \
							if self.is_rotated else  \
							np.array([xg_r, yg_r])

		fov = np.pi/2
		lt = 0.6 #2.

		th_p = fov / 2
		th_m = -fov / 2


		is_success = False

		if  (thg_r > th_m) and \
			(thg_r < th_p) and \
			( np.linalg.norm( [ yg_r, xg_r] ) < lt ):
			is_success = True
			#self.steps = 0
			#self.robot.set_random_goal()


		# Check if robot does not move
		#print('thg_r:{} , xg_r: {}, yg_r:{} '.format(thg_r, xg_r,yg_r))
		
		self.queue_state.append([self.robot.xr, self.robot.yr])

		if all(x == self.queue_state[0] for x in self.queue_state):
			r_stopped = -3
		else:
			r_stopped = 0


		# Rewards 
		r_collide = -100
		r_success = 20
		r_alive = 0.5 + r_stopped
		r_nav = r_alive 

		# Done condition
		done = bool(
			is_crashed 
			or is_success 
			or self.steps > 600
			)
		if not done:
			self.steps += 1
			# Sparse reward exploration
			reward = r_nav
		else:
			if is_crashed: 
				self.final_state = 1
				reward = r_collide
			elif is_success:
				reward = r_success
				self.final_state = 2
				is_success = False
			else: 
				reward = r_nav
				self.final_state = 3
			# Acabó el número de episodios

		return np.concatenate( (robot_pos ,obs)) , reward, done, {}

	def reset(self): # Return to initial state
		self.robot.reset()
		
		self.steps = 0
		self.final_state = 0


		# Save initial pose

		self.xr0 = self.robot.xr
		self.yr0 = self.robot.yr
		self.thr0 = self.robot.thr

		# Transform robot pose in t wrt robot initial frame
		I_T_r = pose2mat_2d(self.robot.xr, self.robot.yr, self.robot.thr)
		I_T_r0 = pose2mat_2d(self.xr0, self.yr0, self.thr0)
		r0_T_I = inverse_mat_2d(I_T_r0)
		r0_T_r = r0_T_I.dot(I_T_r)
		xr_r0, yr_r0, thr_r0 = mat2pose_2d(r0_T_r)		

		if self.is_rotated:
			robot_pos = np.array([xr_r0, yr_r0, thr_r0])
		else:
			robot_pos = np.array([xr_r0, yr_r0])

		r_T_I = inverse_mat_2d(I_T_r)


		# Transform goal position to robot inertia frame
		I_T_g = pose2mat_2d(self.robot.xg, self.robot.yg, self.robot.thg)
		r_T_g = r_T_I.dot(I_T_g)
		xg_r, yg_r, thg_r = mat2pose_2d(r_T_g)

		self.robot_goal = 	np.array([xg_r, yg_r, thg_r]) \
							if self.is_rotated else  \
							np.array([xg_r, yg_r])

		# Transform lidar to robot inertia frame
		touches = self.robot.scanning()
		
		xls = self.robot.xls[touches]
		yls = self.robot.yls[touches]

		if xls.size == 0:
			xls = np.array([self.robot.max_range])
			yls = np.array([0.])

		xls_r = []
		yls_r = []


		for xl, yl in zip(xls, yls):
			I_T_l = pose2mat_2d(xl, yl, 0)
			r_T_l = r_T_I.dot(I_T_l)
			xl_r, yl_r, _ = mat2pose_2d(r_T_l)
			xls_r.append(xl_r)
			yls_r.append(yl_r)

		xls_r = np.array(xls_r)
		yls_r = np.array(yls_r)

		obs = np.concatenate( (xls_r, yls_r) )
		
		#print("The environment has been reset")
		return np.concatenate( (robot_pos ,obs)) 


	def render(self):
		self.robot.render()

	def close(self):
		self.robot.close()


def pose2mat_2d(x, y, th):
	T = np.array([
		[np.cos(th),	-np.sin(th),	x],
		[np.sin(th),	np.cos(th), y],
		[0,				0,			1]]
	)
	return T

def mat2pose_2d(T):
	th = np.arctan2(T[1,0], T[0,0])
	x = T[0,2]
	y = T[1,2]
	return x, y, th

def inverse_mat_2d(T):
	"""
	Calculates the inverse of a 2D Transformation Matrix (3x3).
	"""
	R_in = T[:2,:2]
	t_in = T[:2,[-1]]
	R_out = R_in.T
	t_out = - np.matmul(R_out,t_in)
	return np.vstack((np.hstack((R_out,t_out)),np.array([0, 0, 1])))