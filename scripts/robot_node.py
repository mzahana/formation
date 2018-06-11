#!/usr/bin/env python

import rospy # ROS interface
import pymap3d as pm # coordinate conversion
import numpy as np

from subprocess import call

# msgs
from formation.msg import RobotState, FormationPositions
from std_msgs.msg import Empty, Int32
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# math
from math import atan2, cos, sin

import time

class FcuModes:
	def __init__(self):
		pass

	def setArm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(True)
		except rospy.ServiceException, e:
			print "Service arming call failed: %s"%e

	def setDisarm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(False)
		except rospy.ServiceException, e:
			print "Service disarming call failed: %s"%e

	def setStabilizedMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='STABILIZED')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

	def setOffboardMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='OFFBOARD')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Offboard Mode could not be set."%e

	def setAltitudeMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='ALTCTL')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Altitude Mode could not be set."%e

	def setPositionMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='POSCTL')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Position Mode could not be set."%e

	def setAutoLandMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='AUTO.LAND')
		except rospy.ServiceException, e:
	   		print "service set_mode call failed: %s. Autoland Mode could not be set."%e



class Robot:
	def __init__(self):
		# This robot's ID; starts from zero
		self.myID 				= rospy.get_param("myID",0)

		# number of robots
		self.n					= rospy.get_param("nRobots",5)

		self.origin				= rospy.get_param("origin", [10.12345, 20.12345])
		self.east				= rospy.get_param("east", [10.12345, 20.12345])
		self.h0					= rospy.get_param("h0", 0.0)

		# if indoor position is used
		self.INDOOR				= rospy.get_param("indoor", False)

		# rotation from geo local ENU to desired local ENU
		self.local_rot			= 0.0

		# Altitude at ground (Zero alt)
		self.GND_ALT			= 0.0

		# start position
		self.start_pos			= np.zeros((1,3))

		# current position
		self.curr_lat			= 0.0
		self.curr_lon			= 0.0
		self.gpsAlt				= 0.0
		self.curr_z_enu			= 0.0
		self.curr_yaw			= 0.0

		# current local pose in robot's ENU
		self.local_pose			= PoseStamped()

		# current position in deisred ENU
		self.current_pos		= np.zeros((1,3))
		# target position in desired ENU
		self.target_pos			= np.zeros((1,3))

		# Takeof altitude:
		self.TOALT				= rospy.get_param("TOALT", 1.0)

		# distance threshold
		self.dist_TH			= rospy.get_param("dist_TH", 0.5)

		# Instantiate a setpoint topic structure
		self.mavros_sp			= PositionTarget()
		# use position setpoints
		self.mavros_sp.type_mask= int('101111111000', 2)

		# Disctionary to hold boolean status of commands
		self.cmd				= {"Takeoff": 0, "Land": 0, "Arm": 0, "Disarm": 0, "Hold": 0, "GO": 0}

		# Goal positions nx3
		self.G					= np.zeros((self.n,3))
		#my goal 1x3
		self.my_goal			= np.zeros((1,3))

		# completion time
		self.tf					= 0.0

		# Robot state msg
		self.robot_msg			= RobotState()

		# used to cmopute t0 only once after the GO signal is received
		self.t0_trigger			= False

	#################### Calbacks #####################

	def formationCb(self, msg):
		self.tf = msg.tf
		rospy.logwarn("[Robot %s]: Received Goals", self.myID)
		for i in range(self.n):
			self.G[i,:] = np.array([msg.goals[i].x, msg.goals[i].y, msg.goals[i].z])

		self.my_goal = self.G[self.myID,:]

		self.robot_msg.received_goal = True

		# set start position
		if self.INDOOR:
			x = self.local_pose.pose.position.x
			y = self.local_pose.pose.position.y
			z = self.local_pose.pose.position.z
		else:
			x,y,z = self.geo2desiredENU(self.curr_lat, self.curr_lon, self.gpsAlt)

		self.start_pos = np.array([x,y,z])

		# reset flag
		self.robot_msg.arrived = False

		# reset go_sig
		self.go_sig				= False

	def armCb(self, msg):
		self.cmd = self.cmd.fromkeys(self.cmd, 0)
		self.cmd["Arm"] = 1

	def armAllCb(self, msg):
		self.cmd = self.cmd.fromkeys(self.cmd, 0)
		self.cmd["Arm"] = 1

	def disArmCb(self, msg):
		self.cmd = self.cmd.fromkeys(self.cmd, 0)
		self.cmd["Disarm"] = 1

	def disArmAllCb(self, msg):
		self.cmd = self.cmd.fromkeys(self.cmd, 0)
		self.cmd["Disarm"] = 1

	def takeoffCb(self, msg):
		self.cmd = self.cmd.fromkeys(self.cmd, 0)
		self.cmd["Takeoff"] = 1

	def takeoffAllCb(self, msg):
		self.cmd = self.cmd.fromkeys(self.cmd, 0)
		self.cmd["Takeoff"] = 1

	def landCb(self, msg):
		self.cmd = self.cmd.fromkeys(self.cmd, 0)
		self.cmd["Land"] = 1

	def landAllCb(self, msg):
		self.cmd = self.cmd.fromkeys(self.cmd, 0)
		self.cmd["Land"] = 1

	def holdCb(self, msg):
		self.cmd = self.cmd.fromkeys(self.cmd, 0)
		self.cmd["Hold"] = 1

	def holdAllCb(self, msg):
		self.cmd = self.cmd.fromkeys(self.cmd, 0)
		self.cmd["Hold"] = 1

	def goCb(self, msg):
		self.cmd = self.cmd.fromkeys(self.cmd, 0)
		self.cmd["GO"] = 1

	def gpsCb(self, msg):
		self.curr_lat = msg.latitude
		self.curr_lon = msg.longitude
		self.gpsAlt   = msg.altitude

		# set current position in desired ENU
		if self.INDOOR:
			x = self.local_pose.pose.position.x
			y = self.local_pose.pose.position.y
			z = self.local_pose.pose.position.z
		else:
			x,y,z = self.geo2desiredENU(self.curr_lat, self.curr_lon, self.gpsAlt)

		self.current_pos = np.array([x,y,z])
		self.robot_msg.point.x = x
		self.robot_msg.point.y = y
		self.robot_msg.point.z = z

	def localPoseCb(self, msg):
		self.local_pose = msg
		self.curr_z_enu = msg.pose.position.z

	def setnRobotsCb(self, msg):
		self.n = msg.data

	def originCb(self, msg):
			self.origin[0] = msg.x
			self.origin[1] = msg.y
			self.h0 = msg.z

	def eastCb(self, msg):
			self.east[0] = msg.x
			self.east[1] = msg.y

	def shutdownCb(self, msg):
		call("sudo shutdown -h now", shell=True)

	def rebootCb(self, msg):
		call("sudo reboot", shell=True)
	############### End of Callbacks #################

	def next_sp(self, t):
		if t > self.tf:
			t = self.tf
		self.target_pos = self.start_pos + ( (self.my_goal - self.start_pos)/self.tf ) * t

	def compute_local_roation(self):
		h = self.h0
		lat = self.east[0]
		lon = self.east[1]
		lat0 = self.origin[0]
		lon0 = self.origin[1]
		x,y,z = pm.geodetic2enu(lat, lon, h, lat0, lon0, self.h0)
		self.local_rot = atan2(y, x)
		rospy.logwarn("Local roation = %s", self.local_rot)
		return self.local_rot

	def geo2desiredENU(self, lat, lon, h):
		"""Converts from LLA to desired local ENU coordinates.
		It requires origin LLA and local_rot
		"""
		lat0 = self.origin[0]
		lon0 = self.origin[1]
		x,y,z = pm.geodetic2enu(lat, lon, h, lat0, lon0, self.h0)

		x_L = cos(self.local_rot)*x + sin(self.local_rot)*y
		y_L = -1*sin(self.local_rot)*x + cos(self.local_rot)*y

		z = self.curr_z_enu - self.GND_ALT
		return x_L, y_L, z

	def desiredENU2geo(self, x_L, y_L, z):
		"""Converts from local ENU grid coordinates
		to geoditic LLA.
		It requires local_rot, origin LLA
		"""
		x = cos(self.local_rot)*x_L - sin(self.local_rot)*y_L
		y = sin(self.local_rot)*x_L + cos(self.local_rot)*y_L

		lat0 = self.origin[0]
		lon0 = self.origin[1]

		lat, lon, alt = pm.enu2geodetic(x, y, z, lat0, lon0, self.h0)
		return lat, lon, alt

	def set_robot_pos(self):
		"""converts the current geo location to desired enu
		"""
		x,y,z = self.geo2desiredENU(self.curr_lat, self.curr_lon, self.gpsAlt)
		self.robot_msg.point.x = x
		self.robot_msg.point.y = y
		self.robot_msg.point.z = z

	def desiredENU2localSp(self, xd, yd, zd):
		"""Converts locaion in desired ENU to setpoint in robot's ENU
		"""

		if self.INDOOR:
			self.mavros_sp.position.x = xd
			self.mavros_sp.position.y = yd
			self.mavros_sp.position.z = zd
		else:

			lat, lon, alt = self.desiredENU2geo(xd, yd, zd)
			# converts to differences in robot's ENU
			x,y,z = pm.geodetic2enu(lat, lon, alt, self.curr_lat, self.curr_lon, self.gpsAlt)

			# Add the differences to robot's current location
			self.mavros_sp.position.x = self.local_pose.pose.position.x + x
			self.mavros_sp.position.y = self.local_pose.pose.position.y + y
			self.mavros_sp.position.z = zd + self.GND_ALT

def main():

	rospy.init_node('formation_robot_node', anonymous=True)

	R = Robot()

	rospy.logwarn("Started Robot Node %s", R.myID)

	mode = FcuModes()

	# Subscriber: GPS
	rospy.Subscriber("mavros/global_position/raw/fix", NavSatFix, R.gpsCb)

	# Subscriber: Local pose
	rospy.Subscriber("mavros/local_position/pose", PoseStamped, R.localPoseCb)

	# Subscriber nRobots
	rospy.Subscriber("/setnRobots", Int32, R.setnRobots)

	# Subscriber of origin/east coordinates
	rospy.Subscriber("/setOrigin", Point, R.originCb)
	rospy.Subscriber("/setEast", Point, R.eastCb)

	# Subscribers: Commands
	rospy.Subscriber('takeoff', Empty, R.takeoffCb)
	rospy.Subscriber('/takeoff', Empty, R.takeoffAllCb)
	rospy.Subscriber('land', Empty, R.landCb)
	rospy.Subscriber('/land', Empty, R.landAllCb)
	rospy.Subscriber('arm', Empty, R.armCb)
	rospy.Subscriber('/arm', Empty, R.armAllCb)
	rospy.Subscriber('disarm', Empty, R.disArmCb)
	rospy.Subscriber('/disarm', Empty, R.disArmAllCb)
	rospy.Subscriber('hold', Empty, R.holdCb)
	rospy.Subscriber('/hold', Empty, R.holdAllCb)

	rospy.Subscriber('/go', Empty, R.goCb)

	rospy.Subscriber('/formation', FormationPositions, R.formationCb)

	rospy.Subscriber('shutdown', Empty, R.shutdownCb)
	rospy.Subscriber('/shutdown', Empty, R.shutdownCb)

	rospy.Subscriber('reboot', Empty, R.rebootCb)
	rospy.Subscriber('/reboot', Empty, R.rebootCb)

	# Publisher: PositionTarget
	setp_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=1)
	# robot msg
	robot_pub = rospy.Publisher("state", RobotState, queue_size=1) 

	rate = rospy.Rate(10.0)

	# loop for a bit to get he ground altitude
	k = 0
	while ( not rospy.is_shutdown() and k < 10):
		R.GND_ALT = R.curr_z_enu
		k = k + 1
		rate.sleep()

	# compute local rotation
	R.compute_local_roation()


	rospy.logwarn("Got ground altitude = %s", R.GND_ALT)

	while not rospy.is_shutdown():

		# react to FCU mode requests 
		if R.cmd["Arm"]:
			R.cmd = R.cmd.fromkeys(R.cmd, 0)
			mode.setArm()
			rospy.logwarn("Robot %s is Arming", R.myID)

		if R.cmd["Disarm"]:
			R.cmd = R.cmd.fromkeys(R.cmd, 0)
			mode.setDisarm()
			rospy.logwarn("Robot %s is Disarming", R.myID)

		if R.cmd["Land"]:
			R.cmd = R.cmd.fromkeys(R.cmd, 0)
			mode.setAutoLandMode()
			go_sig = False
			rospy.logwarn("Robot %s is landing", R.myID)

		if R.cmd["Takeoff"]:
			R.mavros_sp.position.z = R.GND_ALT + R.TOALT

			if (R.curr_z_enu - R.GND_ALT) > 1.0:
				rospy.logwarn("Robot %s is already in air", R.myID)
				R.cmd = R.cmd.fromkeys(R.cmd, 0)
			else:
				rospy.logwarn('Robot %s: Arm and Takeoff.', R.myID)
				if (R.curr_z_enu  - R.GND_ALT) < 0.5:
					R.mavros_sp.position.x = R.local_pose.pose.position.x
					R.mavros_sp.position.y = R.local_pose.pose.position.y
				else:
					R.cmd = R.cmd.fromkeys(R.cmd, 0)

				mode.setArm()
				mode.setOffboardMode()

		if R.cmd["GO"] and R.robot_msg.received_goal and not R.robot_msg.arrived:
			R.cmd = R.cmd.fromkeys(R.cmd, 0)
			R.cmd["GO"] = 1
			if not R.t0_trigger:
				t0 = time.time()
				R.t0_trigger = True
			t = time.time()
			R.next_sp(t-t0)
			R.desiredENU2localSp(R.target_pos[0], R.target_pos[1], R.target_pos[2])

			if np.linalg.norm(R.my_goal - R.current_pos) < R.dist_TH:
				R.cmd = R.cmd.fromkeys(R.cmd, 0)
				R.robot_msg.received_goal = False
				R.robot_msg.arrived = True
		elif R.cmd["GO"] and (not R.robot_msg.received_goal or R.robot_msg.arrived):
			R.cmd = R.cmd.fromkeys(R.cmd, 0)
			rospy.logwarn("[Robot %s]: Not executing GO signal. Either Goal is not recieved or already arrived", R.myID)

		R.mavros_sp.header.stamp = rospy.Time.now()
		R.robot_msg.header.stamp = rospy.Time.now()
		setp_pub.publish(R.mavros_sp)
		robot_pub.publish(R.robot_msg)

		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
