#!/usr/bin/env python

import rospy

from formation.msg import RobotState, FormationPositions, RobotTarget
import pymavlinik.mavutil as mavutil


class RobotBridge():

	def __init__(self):
		
		# this robot's ID; starts from 0
		self.myID		= rospy.get_param("myID", 0)

		# pymavlink connection
		self.robot_udp	= rospy.get_param("robot_udp", "127.0.0.1:30000")
		# this robot's mavlink ID = myID+1
		src_sys			= self.myID + 1
		self.mav		= mavutil.mavlink_connection("udpin:"+self.robot_udp, source_system=src_sys)

	##### Callbacks ###

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

