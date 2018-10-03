#!/usr/bin/env python

import rospy # ROS interface
import pymap3d as pm # coordinate conversion

# msgs
from formation.msg import RobotFormationState, FormationPositions
from std_msgs.msg import Empty, Int32
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

# to get ROS package path
import rospkg
rospack = rospkg.RosPack()

class Mission:
	def __init__(self):

		self.DEBUG		= rospy.get_param("DEBUG", False)
		# number of robots
		self.n 			= rospy.get_param("nRobots", 5)
		# robot's radius
		self.R			= rospy.get_param("robot_radius", 0.5)
		# safety distance
		self.delta		= 2.0*(2**0.5)*self.R
		# max robot's velocity
		self.vmax		= rospy.get_param("vmax", 1.0)

		self.origin		= rospy.get_param("origin", [10.12345, 20.12345])
		self.east		= rospy.get_param("east", [10.12345, 20.12345])

		# formation: start position nx3 array
		self.P			= np.zeros((self.n,3))

		# formation: desired shape positions
		self.S			= rospy.get_param("shape")

		# get number of desired shapes
		self.nS			= len(self.S)

		# shape durations
		self.Sdt			= rospy.get_param("durations", [5.0])

		# shape counter: used to transition between shapes in mission
		self.shape_counter	= 0

		# holds one shape at a time
		self.S_array	= np.array(self.S[0])

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

		# Falgs to check if all robots started mission at the same time
		self.robots_started_mission = self.n*[False]

		# True if all robots arrived at their goals
		# True if all elements in self.arrival_state are True
		self.formation_completed = False

		# Mission flags

		# true if P & S are valid
		self.valid_P		= False
		self.valid_S		= False

		self.P_is_set		= False
		self.S_is_set		= False

		self.G_is_set		= False

		self.USER_START		= False # set through user input
		self.M_END			= False

		self.START			= False # set through code only
		self.ASSIGNMENT		= False
		self.GOAL_SEND		= False
		self.GOAL_RECEIVE	= False
		self.SEND_GO		= False
		self.CHECK_SHAPE	= False
		self.STAY_IN_SHAPE	= False


		# Topics names of robots locations in local defined ENU coordiantes
		self.r_loc_topic_names = []
		rstr = "/robot"
		for i in range(self.n):
			self.r_loc_topic_names.append(rstr+str(i)+"/state")


	def validate_positions(self):
		self.valid_P = True
		self.valid_S = True
		for i in range(self.n):
			if not self.valid_P:
				break

			for j in range(self.n):
				if i != j:
					if np.linalg.norm(self.P[i,:] - self.P[j,:]) <= self.delta:
						self.valid_P = False
						rospy.logwarn("Robots %s and %s are not well seperated", i,j)
						print self.P[i,:]
						print self.P[j,:]
						break

		for i in range(self.n):
			if not self.valid_S:
				break

			for j in range(self.n):
				if i != j:
					if np.linalg.norm(self.S_array[i,:] - self.S_array[j,:]) <= self.delta:
						self.valid_S = False
						rospy.logwarn("[Shape %s]: Goals %s and %s are not well seperated", self.shape_counter, i,j)
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

		self.G_is_set = True


		self.trigger_sig = False

		self.shape_is_complete = False

	################# Callbacks ###############

	############### Robots state callback ###############
	def robotStateCb(self, msg):
		if msg is not None:
			i=msg.robot_id
			p = np.array([msg.point.x, msg.point.y, msg.point.z])
			self.current_P[i,:] = p
			self.arrival_state[i] = msg.arrived
			self.received_goals[i] = msg.received_goal
			self.robots_started_mission[i] = msg.mission_started

	def r0Cb(self, msg):
		i=0
		p = np.array([msg.point.x, msg.point.y, msg.point.z])
		self.current_P[i,:] = p
		self.arrival_state[i] = msg.arrived
		self.received_goals[i] = msg.received_goal
		self.robots_started_mission[i] = msg.mission_started

	def r1Cb(self, msg):
		i=1
		p = np.array([msg.point.x, msg.point.y, msg.point.z])
		self.current_P[i,:] = p
		self.arrival_state[i] = msg.arrived
		self.received_goals[i] = msg.received_goal
		self.robots_started_mission[i] = msg.mission_started

	def r2Cb(self, msg):
		i=2
		p = np.array([msg.point.x, msg.point.y, msg.point.z])
		self.current_P[i,:] = p
		self.arrival_state[i] = msg.arrived
		self.received_goals[i] = msg.received_goal
		self.robots_started_mission[i] = msg.mission_started
		
	def r3Cb(self, msg):
		i=3
		p = np.array([msg.point.x, msg.point.y, msg.point.z])
		self.current_P[i,:] = p
		self.arrival_state[i] = msg.arrived
		self.received_goals[i] = msg.received_goal
		self.robots_started_mission[i] = msg.mission_started

	def r4Cb(self, msg):
		i=4
		p = np.array([msg.point.x, msg.point.y, msg.point.z])
		self.current_P[i,:] = p
		self.arrival_state[i] = msg.arrived
		self.received_goals[i] = msg.received_goal
		self.robots_started_mission[i] = msg.mission_started

	def r5Cb(self, msg):
		i=5
		p = np.array([msg.point.x, msg.point.y, msg.point.z])
		self.current_P[i,:] = p
		self.arrival_state[i] = msg.arrived
		self.received_goals[i] = msg.received_goal
		self.robots_started_mission[i] = msg.mission_started

	def setnRobotsCb(self, msg):
		# NOTE: I geuss this parameter should not be redefined in mission. It only should be defined in a parameters file
		# possible fix is to rewrite this parameter in file.
		pkg_path = rospack.get_path('formation')
		file_path = pkg_path+'/config/params.yaml'
		f = open(file_path,"r")
		lines = f.readlines()
		f.close()
		f = open(file_path,"w")

		for line in lines:
		    if 'nRobots' in line:
		        newline = 'nRobots: '+str(msg.data)+'\n'
		        f.write(newline)
		    else:
		        f.write(line)

		f.close()

		rospy.logwarn("Restart Mster nodes and reboot robots to take effect.")

	def originCb(self, msg):
		self.origin[0] = msg.x
		self.origin[1] = msg.y

		# overright paramter in the /config/params.yaml
		pkg_path = rospack.get_path('formation')
		file_path = pkg_path+'/config/params.yaml'
		f = open(file_path,"r")
		lines = f.readlines()
		f.close()
		f = open(file_path,"w")

		for line in lines:
		    if 'origin' in line:
		        newline = 'origin: ['+str(msg.x)+', '+str(msg.y)+']\n'
		        f.write(newline)
		    else:
		        f.write(line)

		f.close()

	def eastCb(self, msg):
		self.east[0] = msg.x
		self.east[1] = msg.y

		# overright paramter in the /config/params.yaml
		pkg_path = rospack.get_path('formation')
		file_path = pkg_path+'/config/params.yaml'
		f = open(file_path,"r")
		lines = f.readlines()
		f.close()
		f = open(file_path,"w")

		for line in lines:
		    if 'east' in line:
		        newline = 'east: ['+str(msg.x)+', '+str(msg.y)+']\n'
		        f.write(newline)
		    else:
		        f.write(line)

		f.close()

	def setVmaxCb(self, msg):
		if msg is not None:
			self.vmax = msg.data

			# overright paramter in the /config/params.yaml
			pkg_path = rospack.get_path('formation')
			file_path = pkg_path+'/config/params.yaml'
			f = open(file_path,"r")
			lines = f.readlines()
			f.close()
			f = open(file_path,"w")

			for line in lines:
			    if 'vmax' in line:
			        newline = 'vmax: '+str(msg.data)+'\n'
			        f.write(newline)
			    else:
			        f.write(line)

			f.close()

	def set_start_posCb(self):
		"""Sets starting position from robot current position
		"""
		self.P = np.copy(self.current_P)
		self.P_is_set = True

	def set_desired_formation(self):
		"""gets desired formation from shape.yaml in config folder
		"""
		self.S = rospy.get_param("shape")
		self.S_array = np.array(self.S[self.shape_counter])
		self.S_is_set = True

	def startCb(self, msg):
		if self.USER_START:
			rospy.logwarn("Mission already started!")
		else:
			#self.start_assignment()
			self.USER_START = True
			self.M_END = False

	def start_assignment(self):
		self.set_start_posCb()
		self.set_desired_formation()
		self.validate_positions()
		if self.valid_S and self.valid_P:
			self.compute_assignment()
			return True
		else:
			#self.USER_START = False
			return False

	def landCb(self, msg):
		# reset flags
		self.M_START = False
		self.M_END = True
	def holdCb(self, msg):
		# reset flags
		self.M_START = False
		self.M_END = True

	def isFormationComplete(self):
		self.formation_completed = all(self.arrival_state)
		return self.formation_completed

def main():
	
	rospy.init_node('formation_master_node', anonymous=True)

	rospy.logwarn("Started Master Node.")

	M = Mission()

	for i in range(M.n):
		rospy.Subscriber(M.r_loc_topic_names[i], RobotFormationState, M.robotStateCb)	

	# Subscriber nRobots
	rospy.Subscriber("/setnRobots", Int32, M.setnRobotsCb)

	# Subscriber of origin/east coordinates
	rospy.Subscriber("/setOrigin", Point, M.originCb)
	rospy.Subscriber("/setEast", Point, M.eastCb)

	# Subscriber: start flag
	rospy.Subscriber("/start", Empty, M.startCb)
	#Subscriber: land/hold
	rospy.Subscriber("/land", Empty, M.landCb)
	rospy.Subscriber("/hold", Empty, M.holdCb)

	# Publisher: Formation goals & tf
	form_pub = rospy.Publisher("/formation", FormationPositions, queue_size=1)
	# Publisher: GO signal
	go_pub = rospy.Publisher("/go", Empty, queue_size=1)
	go_msg = Empty()

	rate = rospy.Rate(10.0)

	# Activiat START state
	M.START = True

	# Flag to print DEBUG messages only ones to avoid flooding the terminal
	DEBUG_FLAG = True

	while not rospy.is_shutdown():

		if M.M_END:
			if M.DEBUG and DEBUG_FLAG:
				rospy.logwarn("[master_node]: Mission END")
				DEBUG_FLAG = False
			M.START = True
			M.USER_START = False
			M.ASSIGNMENT = False
			M.GOAL_SEND = False
			M.SEND_GO = False
			M.CHECK_SHAPE = False
			M.STAY_IN_SHAPE = False
			M.P_is_set = False
			M.S_is_set = False
			M.G_is_set = False
			M.shape_counter= 0



		if M.USER_START and M.START:
			if M.DEBUG and DEBUG_FLAG:
				rospy.logwarn("[master_node]: Mission STARTED")
				DEBUG_FLAG = False
			M.START = False
			M.ASSIGNMENT = True

			DEBUG_FLAG = True

		if M.ASSIGNMENT:
			if M.DEBUG and DEBUG_FLAG:
				rospy.logwarn("[master_node]: Mission in ASSIGNMENT state")
				rospy.logwarn("Processing shape %s", M.shape_counter)
				DEBUG_FLAG = False

			M.ASSIGNMENT = False

			if M.start_assignment():
				M.GOAL_SEND = True
				DEBUG_FLAG = True
			else:
				M.M_END = True
				DEBUG_FLAG = True

		if M.GOAL_SEND:
			if M.DEBUG and DEBUG_FLAG:
				rospy.logwarn("[master_node]: GOAL_SEND state")
				DEBUG_FLAG = False

			if not all(M.received_goals):
				form_pub.publish(M.goals_msg)
			else:
				M.GOAL_SEND = False
				M.SEND_GO = True
				DEBUG_FLAG = True

		if M.SEND_GO:
			if M.DEBUG and DEBUG_FLAG:
				rospy.logwarn("[master_node]: SEND_GO state")
				DEBUG_FLAG = False

			if not all(np.array(M.robots_started_mission) | np.array(M.arrival_state)):
				go_pub.publish(go_msg)
			else:
				M.SEND_GO = False
				M.CHECK_SHAPE = True
				DEBUG_FLAG = True

		if M.CHECK_SHAPE:
			if M.DEBUG and DEBUG_FLAG:
				rospy.logwarn("[master_node]: CHECK_SHAPE state")
				DEBUG_FLAG = False

			if M.isFormationComplete():
				t0 = time.time()
				M.CHECK_SHAPE = False
				M.STAY_IN_SHAPE = True
				DEBUG_FLAG = True

				rospy.logwarn("Shape %s is in place", M.shape_counter)
				if len(M.Sdt) == M.nS:
					dt_shape = M.Sdt[M.shape_counter]
				else:
					dt_shape = M.Sdt[0]

				rospy.logwarn("Waiting %s [seconds] at shape %s.", dt_shape, M.shape_counter)

		if M.STAY_IN_SHAPE:
			if M.DEBUG and DEBUG_FLAG:
				rospy.logwarn("[master_node]: STAY_IN_SHAPE state")
				DEBUG_FLAG = False

			dt = time.time() - t0
			if len(M.Sdt) == M.nS:
				dt_shape = M.Sdt[M.shape_counter]
			else:
				dt_shape = M.Sdt[0]

			if dt > dt_shape:
				M.STAY_IN_SHAPE = False
				M.shape_counter = M.shape_counter + 1

				if M.shape_counter > (M.nS-1):
					M.shape_counter = 0 # reset counter for new mission
					M.M_END = True
					M.START = True # to be ready for new start
					M.USER_START = False # to wait for start signal from user

					DEBUG_FLAG = True

					rospy.logwarn("############ Mission is done! ###############")
				else:
					M.ASSIGNMENT = True
					DEBUG_FLAG = True
					rospy.logwarn("Going to shape %s", M.shape_counter)



		"""

		if M.isFormationComplete():
			M.shape_is_complete = True
			M.M_START = False
			M.P_is_set = False
			M.S_is_set = False
			M.G_is_set = False

		if M.M_START:
			M.M_END = False
			if not all(M.received_goals):
				form_pub.publish(M.goals_msg)
			else:
				if not M.trigger_sig: # to publish go_msg only once
					go_pub.publish(go_msg)
					M.trigger_sig = True

		"""

		rate.sleep()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
