#!/usr/bin/env python

import rospy

from formation.msg import RobotFormationState, FormationPositions, RobotTarget, VehicleState
import pymavlink.mavutil as mavutil
from std_msgs.msg import Empty, Int32, Float32
from geometry_msgs.msg import Point
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticStatus

from threading import Thread

from time import sleep


class RobotBridge():

	def __init__(self):
		
		self.DEBUG					= rospy.get_param("DEBUG", False)

		# this robot's ID; starts from 0
		self.myID					= rospy.get_param("myID", 0)
		if self.DEBUG:
			rospy.logwarn("[DEBUG] Robot ID %s", self.myID)

		self.master_sys_id			= rospy.get_param("master_sys_id", 255)
		if self.DEBUG:
			rospy.logwarn("[Robot %s]: Master MAVLink ID is %s", self.myID, self.master_sys_id)

		# pymavlink connections
		#----------------------
		# Used mainly to exchange direct MAVLink msgs with FCU (via mavros gcs_url for now)
		self.robot_udp_link			= rospy.get_param("robot_udp_link", "127.0.0.1:30000")

		# Serila link used to exchange MAVLink with master node (e.g. using telemetry radio)
		self.robot_serial_link		= rospy.get_param("robot_serial_link","/dev/ttyUSB0")

		self.serial_baudrate		= rospy.get_param("serial_baudrate", 57600)

		# pymavlink connection
		self.robot_udp				= rospy.get_param("robot_udp", "127.0.0.1:30000")
		if self.DEBUG:
			rospy.logwarn("[Robot %s]: robot_udp= %s", self.myID, self.robot_udp)

		# this robot's mavlink ID starts at 1 = myID+1
		self.my_mavlink_ID			= self.myID + 1
		if self.DEBUG:
			rospy.logwarn("[Robot %s]: Robot MAVLink ID = %s", self.myID, self.my_mavlink_ID)

		# link to fcu (e.g. through MAVROS gcs bridge): udp
		self.fcu_link				= mavutil.mavlink_connection("udpin:"+self.robot_udp_link, source_system=self.master_sys_id)

		# link to master: serial
		self.mav 					= mavutil.mavlink_connection(self.robot_serial_link, baud=self.serial_baudrate, source_system=self.my_mavlink_ID)
		#----------------------------
		# DONE with pymavlink connection


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
		self.MASTER_CMD_SET_TOALT	= 15
		self.MASTER_CMD_ACK			= 16
		self.ROBOT_HEALTH_OK		= 17
		self.ROBOT_HEALTH_BAD		= 18
		self.FCU_STATE				= 19
		self.MAV_MODE_MANUAL		= 20
		self.MAV_MODE_OFFBOARD		= 21
		self.MAV_MODE_POSCTL		= 22
		self.MAV_MODE_ALTCTL		= 23
		self.MAV_MODE_RTL			= 24
		self.MAV_MODE_LAND			= 25
		self.MAV_MODE_UNKNOWN		= 26

		# msg to store VehicleState
		self.vehicle_state			= VehicleState()

		# Subscribers
		rospy.Subscriber('state', RobotFormationState, self.stateCb)
		rospy.Subscriber('mavros/state', State, self.mavros_stateCb)
		rospy.Subscriber('mavros/battery', BatteryState, self.batteryCb)
		rospy.Subscriber('mavros/diagnostics', DiagnosticStatus, self.diagnosticCb)

		# Publishers
		self.arm_pub = rospy.Publisher("/arm", Empty, queue_size=1)
		self.disarm_pub = rospy.Publisher("/disarm", Empty, queue_size=1)
		self.tko_pub = rospy.Publisher("/takeoff", Empty, queue_size=1)
		self.land_pub = rospy.Publisher("/land", Empty, queue_size=1)
		self.hold_pub = rospy.Publisher("/hold", Empty, queue_size=1)
		self.posctl_pub = rospy.Publisher("/posctl", Empty, queue_size=1)
		self.shutdown_pub = rospy.Publisher("/shutdown", Empty, queue_size=1)
		self.reboot_pub = rospy.Publisher("/reboot", Empty, queue_size=1)
		self.go_pub = rospy.Publisher("/go", Empty, queue_size=1)
		self.nR_pub = rospy.Publisher("/setnRobots", Int32, queue_size=1)
		self.setOrigin_pub = rospy.Publisher("/setOrigin", Point, queue_size=1)
		self.setEast_pub = rospy.Publisher("/setEast", Point, queue_size=1)
		self.goal_pub = rospy.Publisher("robot_target", RobotTarget, queue_size=1)
		self.toalt_pub = rospy.Publisher("/setTOALT", Float32, queue_size=1)
		

	#### Functions
	def send_vehicle_state(self):
		tgt_comp_id = 0
		p1 = self.FCU_STATE
		p2 = self.my_mavlink_ID
		p3 = 1 if self.vehicle_state.connected else 0	# connected
		p4 = 1 if self.vehicle_state.armed else 0		# armed
		p5 = self.vehicle_state.battery					# 0 < battery< 1, or -1 if invalid
		p6 = self.vehicle_state.flight_mode				# floght mode
		p7 = self.vehicle_state.health					# vehicle health
		self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

	##### Callbacks ###

	def mavros_stateCb(self, msg):
		self.vehicle_state.armed = msg.armed
		self.vehicle_state.connected = msg.connected
		# TODO: sort out how to set flgith modes. They should be numbers not string
		if msg.mode == 'OFFBOARD':
			self.vehicle_state.flight_mode = self.MAV_MODE_OFFBOARD
		elif msg.mode == 'MANUAL' or msg.mode == 'STABILIZED':
			self.vehicle_state.flight_mode = self.MAV_MODE_MANUAL
		elif msg.mode == 'POSCTL':
			self.vehicle_state.flight_mode = self.MAV_MODE_POSCTL
		elif msg.mode == 'ALTCTL':
			self.vehicle_state.flight_mode = self.MAV_MODE_ALTCTL
		elif msg.mode == 'AUTO.RTL':
			self.vehicle_state.flight_mode = self.MAV_MODE_RTL
		elif msg.mode == 'AUTO.LAND':
			self.vehicle_state.flight_mode = self.MAV_MODE_LAND
		else:
			self.vehicle_state.flight_mode = self.MAV_MODE_UNKNOWN

	def batteryCb(self, msg):
		if msg.percentage > 0.0:
			self.vehicle_state.battery = msg.percentage
		else:
			self.vehicle_state.battery = -1.0

	def diagnosticCb(self, msg):
		if msg.level > 0:
			self.vehicle_state.health = self.ROBOT_HEALTH_BAD
		else:
			self.vehicle_state.health = self.ROBOT_HEALTH_OK

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
		self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

	def fcu_recvCb(self):
		while(True):
			msg = self.fcu_link.recv_match(blocking=True)
			if msg is not None:
				print "Got MAVLink msg from FCU through UDP mavros link", msg.get_srcSystem()

	def master_recvCb(self):
		# This will be running in a Thread not as ROS callback
		cmd = mavutil.mavlink.MAV_CMD_USER_1
		while(True):
			msg = self.mav.recv_match(blocking=True)
			if msg is not None:
				if msg.get_srcSystem() == self.master_sys_id:
					# make sure it's the right mavlink message and directed to this robot or all robots

					#check for HEARTBEAT msg
					if msg.get_type() == "HEARTBEAT":
						if self.DEBUG:
							rospy.logwarn("Got HEARTBEAT from master GCS and forwarding it to FCU")

						self.fcu_link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, mavutil.mavlink.MAV_MODE_MANUAL_ARMED, 0, mavutil.mavlink.MAV_STATE_ACTIVE)

					if msg.get_type() == "COMMAND_LONG" and msg.command == cmd and (msg.target_system == self.my_mavlink_ID or msg.target_system == 0) and msg.param1 == self.MASTER_CMD:
						# message parsing: param1 is used to differntiate between ROBOT_STATE and MASTER_CMD
						if msg.param2 == self.MASTER_CMD_ARM and msg.param3 == 1:
							if self.DEBUG:
								rospy.logwarn("[Robot %s]: Got Arm CMD", self.myID)
							empty_msg = Empty()
							self.arm_pub.publish(empty_msg)
							# Send ACK to MASTER
							p1 = self.MASTER_CMD_ACK
							p2 = self.MASTER_CMD_ARM
							p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
							tgt_comp_id = 0
							self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

						if msg.param2 == self.MASTER_CMD_ARM and msg.param3 == 0:
							if self.DEBUG:
								rospy.logwarn("[Robot %s]: Got Disarm CMD", self.myID)
							empty_msg = Empty()
							self.disarm_pub.publish(empty_msg)
							# Send ACK to MASTER
							p1 = self.MASTER_CMD_ACK
							p2 = self.MASTER_CMD_ARM
							p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
							tgt_comp_id = 0
							self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

						if msg.param2 == self.MASTER_CMD_TKO:
							if self.DEBUG:
								rospy.logwarn("[Robot %s]: Got Takeoff CMD", self.myID)
							empty_msg = Empty()
							self.tko_pub.publish(empty_msg)
							# Send ACK to MASTER
							p1 = self.MASTER_CMD_ACK
							p2 = self.MASTER_CMD_TKO
							p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
							tgt_comp_id = 0
							self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

						if msg.param2 == self.MASTER_CMD_LAND:
							if self.DEBUG:
								rospy.logwarn("[Robot %s]: Got Land CMD", self.myID)
							empty_msg = Empty()
							self.land_pub.publish(empty_msg)
							# Send ACK to MASTER
							p1 = self.MASTER_CMD_ACK
							p2 = self.MASTER_CMD_LAND
							p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
							tgt_comp_id = 0
							self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

						if msg.param2 == self.MASTER_CMD_POSCTL:
							if self.DEBUG:
								rospy.logwarn("[Robot %s]: Got POSCTL CMD", self.myID)
							empty_msg = Empty()
							self.posctl_pub.publish(empty_msg)
							# Send ACK to MASTER
							p1 = self.MASTER_CMD_ACK
							p2 = self.MASTER_CMD_POSCTL
							p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
							tgt_comp_id = 0
							self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

						if msg.param2 == self.MASTER_CMD_HOLD:
							if self.DEBUG:
								rospy.logwarn("[Robot %s]: Got Hold CMD", self.myID)
							empty_msg = Empty()
							self.hold_pub.publish(empty_msg)
							# Send ACK to MASTER
							p1 = self.MASTER_CMD_ACK
							p2 = self.MASTER_CMD_HOLD
							p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
							tgt_comp_id = 0
							self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

						if msg.param2 == self.MASTER_CMD_SHUTDOWN:
							if self.DEBUG:
								rospy.logwarn("[Robot %s]: Got Shutdown CMD", self.myID)
							empty_msg = Empty()
							self.shutdown_pub.publish(empty_msg)
							# Send ACK to MASTER
							p1 = self.MASTER_CMD_ACK
							p2 = self.MASTER_CMD_SHUTDOWN
							p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
							tgt_comp_id = 0
							self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

						if msg.param2 == self.MASTER_CMD_REBOOT:
							if self.DEBUG:
								rospy.logwarn("[Robot %s]: Got Reboot CMD", self.myID)
							empty_msg = Empty()
							self.reboot_pub.publish(empty_msg)
							# Send ACK to MASTER
							p1 = self.MASTER_CMD_ACK
							p2 = self.MASTER_CMD_REBOOT
							p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
							tgt_comp_id = 0
							self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

						if msg.param2 == self.MASTER_CMD_GO:
							if self.DEBUG:
								rospy.logwarn("[Robot %s]: Got Go CMD", self.myID)
							empty_msg = Empty()
							self.go_pub.publish(empty_msg)
							# Send ACK to MASTER
							p1 = self.MASTER_CMD_ACK
							p2 = self.MASTER_CMD_GO
							p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
							tgt_comp_id = 0
							self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

						if msg.param2 == self.MASTER_CMD_SET_nROBOTS:
							if self.DEBUG:
								rospy.logwarn("[Robot %s]: Got SET_nROBOTS= %s CMD", self.myID, msg.param3)
							int_msg = Int32()
							int_msg.data = msg.param3
							self.nR_pub.publish(int_msg)
							# Send ACK to MASTER
							p1 = self.MASTER_CMD_ACK
							p2 = self.MASTER_CMD_SET_nROBOTS
							p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
							tgt_comp_id = 0
							self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

						if msg.param2 == self.MASTER_CMD_SET_ORIGIN:
							if self.DEBUG:
								rospy.logwarn("[Robot %s]: Got SET_ORIGIN=(%s, %s) CMD", self.myID, msg.param3, msg.param4)
							point_msg = Point()
							point_msg.x = msg.param3
							point_msg.y = msg.param4
							point_msg.z = msg.param5
							self.setOrigin_pub.publish(point_msg)
							# Send ACK to MASTER
							p1 = self.MASTER_CMD_ACK
							p2 = self.MASTER_CMD_SET_ORIGIN
							p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
							tgt_comp_id = 0
							self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

						if msg.param2 == self.MASTER_CMD_SET_EAST:
							if self.DEBUG:
								rospy.logwarn("[Robot %s]: Got SET_EAST=(%s, %s) CMD", self.myID, msg.param3, msg.param4)
							point_msg = Point()
							point_msg.x = msg.param3
							point_msg.y = msg.param4
							point_msg.z = msg.param5
							self.setEast_pub.publish(point_msg)
							# Send ACK to MASTER
							p1 = self.MASTER_CMD_ACK
							p2 = self.MASTER_CMD_SET_EAST
							p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
							tgt_comp_id = 0
							self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

						if msg.param2 == self.MASTER_CMD_GOAL and msg.target_system == self.my_mavlink_ID:
							if self.DEBUG:
								rospy.logwarn("[Robot %s]: Got GOAL=(%s, %s, %s) CMD, tf=%s", self.myID, msg.param3, msg.param4, msg.param5, msg.param6)
							r_msg = RobotTarget()
							r_msg.robot_id = msg.target_system
							r_msg.goal.x = msg.param3
							r_msg.goal.y = msg.param4
							r_msg.goal.z = msg.param5
							r_msg.tf     = msg.param6
							r_msg.header.stamp = rospy.Time.now()
							self.goal_pub.publish(r_msg)
							# Send ACK to MASTER
							p1 = self.MASTER_CMD_ACK
							p2 = self.MASTER_CMD_GOAL
							p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
							tgt_comp_id = 0
							self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

						if msg.param2 == self.MASTER_CMD_SET_TOALT:
							if self.DEBUG:
								rospy.logwarn("[Robot %s]: Got set SET_TOALT CMD. TOALT=%s", self.myID, msg.param3)
							r_msg = Float32()
							r_msg.data = msg.param3
							self.toalt_pub.publish(r_msg)
							# Send ACK to MASTER
							p1 = self.MASTER_CMD_ACK
							p2 = self.MASTER_CMD_SET_TOALT
							p3, p4, p5, p6, p7 = 0, 0, 0, 0, 0
							tgt_comp_id = 0
							self.mav.mav.command_long_send(self.master_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_USER_1, 0, p1, p2, p3, p4, p5, p6, p7)

				else:
					rospy.logwarn("Got msg from non-master: %s", msg.get_srcSystem())

			# Should we sleep before we poll the udp again?
			sleep(0.02)

def main():

	rospy.init_node('robot_mavlink_node', anonymous=True)

	R = RobotBridge()

	rospy.logwarn("Started robot_mavlink_node for Robot %s", R.myID)

	# Run recevCb in a thread
	recvthread = Thread(target=R.master_recvCb)
	recvthread.daemon = True
	recvthread.start()

	# Run fcu_recvCb in a thread
	fcu_recvCb_thread = Thread(target=R.fcu_recvCb)
	fcu_recvCb_thread.daemon = True
	fcu_recvCb_thread.start()

	rate = rospy.Rate(5.0)

	while not rospy.is_shutdown():
		R.vehicle_state.header.stamp = rospy.Time.now()
		R.send_vehicle_state()
		rate.sleep()

	

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
