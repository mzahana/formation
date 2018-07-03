#!/usr/bin/env python

import rospy

from formation.msg import RobotState, FormationPositions, RobotTarget
import pymavlinik.mavutil as mavutil

class MasterBridge():

	def __init__(self):
		
		# pymavlink connection
		self.master_udp	= rospy.get_param("master_udp", "127.0.0.1:30000")
		self.mav		= mavutil.mavlink_connection("udpout:"+self.master_udp)

	##### Callbacks ###

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

