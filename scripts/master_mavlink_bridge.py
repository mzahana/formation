#!/usr/bin/env python

import rospy

from std_msgs.msg import Empty, Int32
from geometry_msgs.msg import Point
from formation.msg import RobotState, FormationPositions, RobotTarget
import pymavlink.mavutil as mavutil

from threading import Thread

from time import sleep

class MasterBridge():

	def __init__(self):
		
		self.n 						= rospy.get_param("nRobots", 5)

		self.master_sys_id			= rospy.get_param("master_sys_id", 255)

		# pymavlink connection
		self.master_udp				= rospy.get_param("master_udp", "127.0.0.1:30000")
		self.mav					= mavutil.mavlink_connection("udpout:"+self.master_udp)

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

		# Topics names of robots locations in local defined ENU coordiantes
		self.r_loc_topic_names = []
		rstr = "/robot"
		for i in range(self.n):
			self.r_loc_topic_names.append(rstr+str(i)+"/state")

		# Subscribers
		rospy.Subscriber('/arm_robot', Int32, self.armCb)
		rospy.Subscriber('/disarm_robot', Int32, self.disarmCb)
		rospy.Subscriber('/takeoff_robot', Int32, self.tkoCb)
		rospy.Subscriber('/land_robot', Int32, self.landCb)
		rospy.Subscriber('/hold_robot', Int32, self.holdCb)
		rospy.Subscriber('/posctl_robot', Int32, self.posctlCb)
		rospy.Subscriber('/posctl_robot', Int32, self.posctlCb)

		rospy.Subscriber('/formation', FormationPositions, self.formationCb)
		rospy.Subscriber('/go', Empty, self.goCb)

		rospy.Subscriber('/setnRobots', Int32, self.nRCb)
		rospy.Subscriber('/setOrigin', Point, self.setOriginCb)
		rospy.Subscriber('/setEast', Point, self.setEastCb)


		# Publishers
		self.robot_state_pub_list = []
		for i in range(self.n):
			self.robot_state_pub_list.append(rospy.Publisher(self.r_loc_topic_names[i], RobotState, queue_size=1))

	def recvCb(self):
		# This callback will be running inside a Thread

		while (True):
			cmd = mavutil.mavlink.MAV_CMD_USER_1
			msg = self.mav.recv_match(blocking=True)
			if msg is not None:
				cmd_type = msg.get_type()
				src_sys = msg.get_srcSystem()
				msg_tgt = msg.target_system
				if cmd_type == "COMMAND_LONG" and src_sys > 0 and src_sys <= self.n and msg.command == cmd and msg_tgt == self.master_sys_id and msg.param1 == self.ROBOT_STATE:
					state_msg = RobotState()
					state_msg.header.stamp		= rospy.Time.now()
					state_msg.received_goal		= msg.param2
					state_msg.mission_started	= msg.param3
					state_msg.arrived			= msg.param4
					state_msg.point.x			= msg.param5
					state_msg.point.y			= msg.param6
					state_msg.point.z			= msg.param7

					# publish msg to ROS
					self.robot_state_pub_list[src_sys-1].publish(state_msg)

			# Should we sleep before we poll the udp again?
			sleep(0.02)


	##### Callbacks ###

	def goCb(self, msg):
		tgt_sys = 0 # to all
		tgt_comp_id = 0
		p1 = self.MASTER_CMD
		p2 = self.MASTER_CMD_GO
		p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
		self.mav.mav.command_long_send(tgt_sys, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

	def formationCb(self, msg):

		tgt_comp_id = 0
		for i in range(self.n):
			tgt_sys = i+1
			p1 = self.MASTER_CMD
			p2 = self.MASTER_CMD_GOAL
			p3 = msg.goals[i].x
			p4 = msg.goals[i].y
			p5 = msg.goals[i].z
			p6 = msg.tf
			p7 = 0
			self.mav.mav.command_long_send(tgt_sys, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

			sleep(0.01)


	def nRCb(self, msg):
		r_id = 0 # to all
		nR = msg.data
		tgt_comp_id = 0
		p1 = self.MASTER_CMD
		p2 = self.MASTER_CMD_SET_nROBOTS
		p3 = nR # number of robots
		p4, p5, p6, p7 = 0, 0, 0, 0
		self.mav.mav.command_long_send(r_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

	def setOriginCb(self, msg):
		r_id = 0
		tgt_comp_id = 0
		p1 = self.MASTER_CMD
		p2 = self.MASTER_CMD_SET_ORIGIN
		p3 = msg.x 
		p4 = msg.y
		p5 = msg.z
		p6, p7 = 0, 0
		self.mav.mav.command_long_send(r_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

	def setEastCb(self, msg):
		r_id = 0
		tgt_comp_id = 0
		p1 = self.MASTER_CMD
		p2 = self.MASTER_CMD_SET_EAST
		p3 = msg.x 
		p4 = msg.y
		p5 = msg.z
		p6, p7 = 0, 0
		self.mav.mav.command_long_send(r_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

	def armCb(self, msg):
		r_id = msg.data
		tgt_comp_id = 0
		p1 = self.MASTER_CMD
		p2 = self.MASTER_CMD_ARM
		p3 = 1 # 1: arm, 0: disarm
		p4, p5, p6, p7 = 0, 0, 0, 0
		self.mav.mav.command_long_send(r_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

	def disarmCb(self, msg):
		r_id = msg.data
		tgt_comp_id = 0
		p1 = self.MASTER_CMD
		p2 = self.MASTER_CMD_ARM
		p3 = 0 # 1: arm, 0: disarm
		p4, p5, p6, p7 = 0, 0, 0, 0
		self.mav.mav.command_long_send(r_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

	def tkoCb(self, msg):
		r_id = msg.data
		tgt_comp_id = 0
		p1 = self.MASTER_CMD
		p2 = self.MASTER_CMD_TKO
		p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
		self.mav.mav.command_long_send(r_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

	def landCb(self, msg):
		r_id = msg.data
		tgt_comp_id = 0
		p1 = self.MASTER_CMD
		p2 = self.MASTER_CMD_LAND
		p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
		self.mav.mav.command_long_send(r_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

	def holdCb(self, msg):
		r_id = msg.data
		tgt_comp_id = 0
		p1 = self.MASTER_CMD
		p2 = self.MASTER_CMD_HOLD
		p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
		self.mav.mav.command_long_send(r_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

	def posctlCb(self, msg):
		r_id = msg.data
		tgt_comp_id = 0
		p1 = self.MASTER_CMD
		p2 = self.MASTER_CMD_POSCTL
		p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
		self.mav.mav.command_long_send(r_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

def main():
	rospy.init_node('master_mavlink_node', anonymous=True)

	M = MasterBridge()

	# Run recevCb in a thread
	recvthread = Thread(target=M.recvCb)
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

