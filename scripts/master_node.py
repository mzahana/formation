#!/usr/bin/env python

import rospy # ROS interface
import pymap3d as pm # coordinate conversion

# msgs
from formation.msg import RobotState, FormationPositions
from std_msgs.msg import Empty
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import *

# math
from math import atan2, cos, sin

# To calculate execution time
import time

# for matrix operation
import numpy as np

# To compute distance matrix
from scipy.spatial import distance

# faster linear sum assignment solver: https://github.com/gatagat/lap
# pip install lap
from lap import lapjv

class Mission:
	def __init__(self):
		self.n 			= rospy.get_param("nRobots", 5)
		self.R			= rospy.get_param("R", 0.5)
		self.delta		= 2.0*(2**0.5)*self.R
		self.vmax		= rosp.get_param("vmax", 1.0)
		self.origin		= rospy.get_param("origin", 5)
		self.east		= rospy.get_param("east", 5)

		# formation: start position nx3 array
		self.P			= np.zeros((self.n,3))
		# formation: desired shape positions
		self.S			= rospy.get_param("shape")
		self.S_array	= np.array(self.S)
		# formation: goal positions nx3 array
		self.G			= np.zeros((self.n,3))

		# current robot positions nx3
		self.current_P	= np.zeros((self.n,3))

		# Assigned goals & completion time
		self.goals_msg 	= FormationPositions()
		p = Point(0.0, 0.0, 0.0)
		for i in range(self.n):
			self.goals_msg.goals.append(p)

		# arrival (to goals) state for all robots
		self.arrival_state = self.n*[False]

		# if all each robot received assigned goal
		self.received_goals = self.n*[False]

		self.formation_completed = False

		# true if P & S are valid
		self.valid_P		= False
		self.valid_S		= False


		# Topics names of robots locations in local defined ENU coordiantes
		self.r_loc_topic_names = []
		rstr = "/robot"
		for i in range(self.n):
			self.r_loc_topic_names.append(rstr+str(i+1)+"/local_pos")


	def validate_positions(self):
		self.valid_P = True
		self.valid_S = True
		for i in range(self.n):
			if not self.valid_P:
				break

			for j in range(n):
				if i != j:
					if np.linalg.norm(self.P[i,:] - self.P[j,:]) <= self.delta:
						self.valid_P = False
						rospy.logwarn("Robots %s and %s are not well seperated", i,j)
						break

		for i in range(self.n):
			if not self.valid_S:
				break

			for j in range(n):
				if i != j:
					if np.linalg.norm(self.S_array[i,:] - self.S_array[j,:]) <= self.delta:
						self.valid_S = False
						rospy.logwarn("Goals %s and %s are not well seperated", i,j)
						break

	def compute_assignment(self):
		# compute sudo cost
		K = -1.0*np.dot(self.P,np.transpose(self.S_array))

		#find optimal assignment LSA
		rospy.logwarn("Solving Assignement Problem....")
		t1 = time.time()
		k_opt, x,y = lapjv(K)
		t2 = time.time()
		rospy.logwarn("Assignment is solved in %s seconds", t2-t1)

		# set goals
		tf = 0.0
		for i in range(self.n):
			self.G[i,:] = self.S_array[x[i],:]
			p = Point(self.G[i,0], self.G[i,1], self.G[i,2])
			self.goals_msg.goals[i] = p

			v = np.linalg.norm(self.P[i,:] - self.G[i,:])/self.vmax
			if v > tf:
				tf = v

		# set completion time, tf
		self.goals_msg.tf = tf
		# update time stamp
		self.goals_msg.header.stamp = rospy.Time.now()

	################# Callbacks
	def r0Cb(self, msg):
		i=0
		p = np.array([msg.point.x, msg.point.y, msg.point.z])
		self.current_P[i,:] = p

	def r1Cb(self, msg):
		i=1
		p = np.array([msg.point.x, msg.point.y, msg.point.z])
		self.current_P[i,:] = p

	def r2Cb(self, msg):
		i=2
		p = np.array([msg.point.x, msg.point.y, msg.point.z])
		self.current_P[i,:] = p
		
	def r3Cb(self, msg):
		i=3
		p = np.array([msg.point.x, msg.point.y, msg.point.z])
		self.current_P[i,:] = p

	def r4Cb(self, msg):
		i=4
		p = np.array([msg.point.x, msg.point.y, msg.point.z])
		self.current_P[i,:] = p

	def set_start_posCb(self, msg):
		"""Sets starting position from robot current position
		"""
		self.P = np.copy(self.current_P)

def main():

	M = Mission()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass