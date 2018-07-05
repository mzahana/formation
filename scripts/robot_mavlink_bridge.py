#!/usr/bin/env python

import rospy

from formation.msg import RobotState, FormationPositions, RobotTarget
import pymavlinik.mavutil as mavutil
from std_msgs.msg import Empty, Int32
from geometry_msgs.msg import Point

from threading import Thread

from time import sleep


class RobotBridge():

	def __init__(self):
		
		self.DEBUG					= rospy.get_param("DEBUG", False)

		# this robot's ID; starts from 0
		self.myID					= rospy.get_param("myID", 0)
		if self.DEBUG:
			rospy.logwarn("[DEBUG] Robot ID %s", self.myID)

		# pymavlink connection
		self.robot_udp				= rospy.get_param("robot_udp", "127.0.0.1:30000")
		if self.DEBUG:
			rospy.logwarn("[Robot %s]: robot_udp= %s", self.myID, self.robot_udp)

		# this robot's mavlink ID = myID+1
		self.my_mavlink_ID			= self.myID + 1
		if self.DEBUG:
			rospy.logwarn("[Robot %s]: Robot MAVLink ID = %s", self.myID, self.my_mavlink_ID)

		self.mav					= mavutil.mavlink_connection("udpin:"+self.robot_udp, source_system=self.my_mavlink_ID)

		self.master_sys_id			= rospy.get_param("master_sys_id", 255)

		# Disctionary to hold boolean status of commands
		self.cmd					= {"Takeoff": 0, "Land": 0, "Arm": 0, "Disarm": 0, "Hold": 0, "GO": 0, "POSCTL": 0}

		self.ROBOT_STATE			= 1
		self.MASTER_CMD				= 2
		self.MASTER_CMD_ARM			= 3
		self.MASTER_CMD_TKO			= 4
		self.MASTER_CMD_LAND		= 5
		self.MASTER_CMD_POSCTL		= 6
		self.MASTER_CMD_HOLD		= 7
		self.MASTER_CMD_SHUTDOWN	= 8
		self.MASTER_CMD_REBOOT		= 9
		self.MASTER_CMD_SET_ORIGIN	= 10
		self.MASTER_CMD_SET_EAST	= 11
		self.MASTER_CMD_SET_nROBOTS	= 12
		self.MASTER_CMD_GO			= 13
		self.MASTER_CMD_GOAL		= 14

		# Subscribers
		rospy.Subscriber('state', RobotState, self.stateCb)

		# Publishers
		self.arm_pub = rospy.Publisher("/arm", Empty, queue_size=1)
		self.disarm_pub = rospy.Publisher("/disarm", Empty, queue_size=1)
		self.tko_pub = rospy.Publisher("/takeoff", Empty, queue_size=1)
		self.land_pub = rospy.Publisher("/land", Empty, queue_size=1)
		self.hold_pub = rospy.Publisher("/hold", Empty, queue_size=1)
		self.shutdown_pub = rospy.Publisher("/shutdown", Empty, queue_size=1)
		self.reboot_pub = rospy.Publisher("/reboot", Empty, queue_size=1)
		self.go_pub = rospy.Publisher("/go", Empty, queue_size=1)
		self.nR_pub = rospy.Publisher("/setnRobots", Int32, queue_size=1)
		self.setOrigin_pub = rospy.Publisher("/setOrigin", Point, queue_size=1)
		self.setEast_pub = rospy.Publisher("/setEast", Point, queue_size=1)
		self.goal_pub = rospy.Publisher("robot_target", RobotTarget, queue_size=1)
		

	##### Callbacks ###
	def stateCb(self, msg):

		# Send robot state as MAV_CMD
		tgt_comp_id = 0
		p1 = self.ROBOT_STATE
		p2 = 1 if msg.received_goal else 0
		p3 = 1 if msg.mission_started else 0
		p4 = 1 if msg.arrived else 0
		p5 = msg.point.x
		p6 = msg.point.y
		p7 = msg.point.z
		self.mav.mav.command_long_send(self.tgt_sys, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

	def recvCb(self):
		# This will be running in a Thread not as ROS callback
		cmd = mavutil.mavlink.MAV_CMD_USER_1
		while(True):
			msg = mav.recv_match(blocking=True)
			if msg is not None:
				if msg.get_srcSystem() == self.master_sys_id:
					# make sure it's the right mavlink message and directed to this robot or all robots
					if msg.get_type() == "COMMAND_LONG" and msg.command == cmd and (msg.target_system == self.my_mavlink_ID or msg.target_system == 0) and msg.param1 == self.MASTER_CMD:
						# message parsing: param1 is used to differntiate between ROBOT_STATE and MASTER_CMD
						if msg.param2 == self.MASTER_CMD_ARM and msg.param3 == 1:
							empty_msg = Empty()
							self.arm_pub.publish(empty_msg)
						if msg.param2 == self.MASTER_CMD_ARM and msg.param3 == 0:
							empty_msg = Empty()
							self.disarm_pub.publish(empty_msg)
						if msg.param2 == self.MASTER_CMD_TKO:
							empty_msg = Empty()
							self.tko_pub.publish(empty_msg)
						if msg.param2 == self.MASTER_CMD_LAND:
							empty_msg = Empty()
							self.land_pub.publish(empty_msg)
						if msg.param2 == self.MASTER_CMD_HOLD
							empty_msg = Empty()
							self.hold_pub.publish(empty_msg)
						if msg.param2 == self.MASTER_CMD_SHUTDOWN:
							empty_msg = Empty()
							self.shutdown_pub.publish(empty_msg)
						if msg.param2 == self.MASTER_CMD_REBOOT:
							empty_msg = Empty()
							self.reboot_pub.publish(empty_msg)
						if msg.param2 == self.MASTER_CMD_GO:
							empty_msg = Empty()
							self.go_pub.publish(empty_msg)
						if msg.param2 == self.MASTER_CMD_SET_nROBOTS:
							int_msg = Int32()
							int_msg.data = msg.param3
							self.nR_pub.publish(int_msg)
						if msg.param2 == self.MASTER_CMD_SET_ORIGIN:
							point_msg = Point()
							point_msg.x = msg.param3
							point_msg.y = msg.param4
							point_msg.z = msg.param5
							self.setOrigin_pub.publish(point_msg)
						if msg.param2 == self.MASTER_CMD_SET_EAST:
							point_msg = Point()
							point_msg.x = msg.param3
							point_msg.y = msg.param4
							point_msg.z = msg.param5
							self.setEast_pub.publish(point_msg)
						if msg.param2 == self.MASTER_CMD_GOAL and msg.target_system == self.my_mavlink_ID:
							r_msg = RobotTarget()
							r_msg.robot_id = msg.target_system
							r_msg.goal.x = msg.param3
							r_msg.goal.y = msg.param4
							r_msg.goal.z = msg.param5
							r_msg.tf     = msg.param6
							r_msg.header.stamp = rospy.Time.now()
							self.goal_pub.publish(r_msg)

				else:
					rospy.logwarn("Got msg from non-master: %s", msg.get_srcSystem())

			# Should we sleep before we poll the udp again?
			sleep(0.02)

def main():

	rospy.init_node('robot_mavlink_node', anonymous=True)

	R = RobotBridge()

	# Run recevCb in a thread
	recvthread = Thread(target=R.recvCb)
	recvthread.daemon = True
	recvthread.start()

	rate = rospy.Rate(5.0)

	while not rospy.is_shutdown():
		rate.sleep()

	

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
