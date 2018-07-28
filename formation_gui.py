# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'formation-gui.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

# ------------------ Added by user ------------------ 
import rospy
from std_msgs.msg import Empty, Int32, Float32
from geometry_msgs.msg import Point
from formation.msg import MultiVehicleState

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(400, 778)
        self.label = QtGui.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(70, 10, 301, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName(_fromUtf8("label"))
        self.groupBox = QtGui.QGroupBox(Form)
        self.groupBox.setGeometry(QtCore.QRect(20, 40, 361, 81))
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.nRobotsEdit = QtGui.QLineEdit(self.groupBox)
        self.nRobotsEdit.setEnabled(False)
        self.nRobotsEdit.setGeometry(QtCore.QRect(130, 30, 113, 20))
        font = QtGui.QFont()
        font.setPointSize(11)
        font.setBold(True)
        font.setWeight(75)
        self.nRobotsEdit.setFont(font)
        self.nRobotsEdit.setFrame(True)
        self.nRobotsEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.nRobotsEdit.setReadOnly(False)
        self.nRobotsEdit.setObjectName(_fromUtf8("nRobotsEdit"))
        self.groupBox_2 = QtGui.QGroupBox(Form)
        self.groupBox_2.setGeometry(QtCore.QRect(20, 140, 361, 131))
        self.groupBox_2.setObjectName(_fromUtf8("groupBox_2"))
        self.label_2 = QtGui.QLabel(self.groupBox_2)
        self.label_2.setGeometry(QtCore.QRect(10, 20, 341, 41))
        self.label_2.setWordWrap(True)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.label_3 = QtGui.QLabel(self.groupBox_2)
        self.label_3.setGeometry(QtCore.QRect(50, 70, 83, 16))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.label_4 = QtGui.QLabel(self.groupBox_2)
        self.label_4.setGeometry(QtCore.QRect(150, 70, 84, 16))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.label_5 = QtGui.QLabel(self.groupBox_2)
        self.label_5.setGeometry(QtCore.QRect(260, 70, 49, 16))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.system_connection_status = QtGui.QLineEdit(self.groupBox_2)
        self.system_connection_status.setEnabled(False)
        self.system_connection_status.setGeometry(QtCore.QRect(50, 98, 71, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        font.setBold(True)
        font.setWeight(75)
        self.system_connection_status.setFont(font)
        self.system_connection_status.setFrame(True)
        self.system_connection_status.setAlignment(QtCore.Qt.AlignCenter)
        self.system_connection_status.setReadOnly(False)
        self.system_connection_status.setObjectName(_fromUtf8("system_connection_status"))
        self.system_armed_status = QtGui.QLineEdit(self.groupBox_2)
        self.system_armed_status.setEnabled(False)
        self.system_armed_status.setGeometry(QtCore.QRect(150, 98, 71, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        font.setBold(True)
        font.setWeight(75)
        self.system_armed_status.setFont(font)
        self.system_armed_status.setFrame(True)
        self.system_armed_status.setAlignment(QtCore.Qt.AlignCenter)
        self.system_armed_status.setReadOnly(False)
        self.system_armed_status.setObjectName(_fromUtf8("system_armed_status"))
        self.system_health_status = QtGui.QLineEdit(self.groupBox_2)
        self.system_health_status.setEnabled(False)
        self.system_health_status.setGeometry(QtCore.QRect(250, 98, 71, 21))
        font = QtGui.QFont()
        font.setPointSize(11)
        font.setBold(True)
        font.setWeight(75)
        self.system_health_status.setFont(font)
        self.system_health_status.setFrame(True)
        self.system_health_status.setAlignment(QtCore.Qt.AlignCenter)
        self.system_health_status.setReadOnly(False)
        self.system_health_status.setObjectName(_fromUtf8("system_health_status"))
        self.groupBox_3 = QtGui.QGroupBox(Form)
        self.groupBox_3.setGeometry(QtCore.QRect(20, 280, 361, 201))
        self.groupBox_3.setObjectName(_fromUtf8("groupBox_3"))
        self.label_6 = QtGui.QLabel(self.groupBox_3)
        self.label_6.setGeometry(QtCore.QRect(30, 30, 83, 16))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.label_6.setFont(font)
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.label_7 = QtGui.QLabel(self.groupBox_3)
        self.label_7.setGeometry(QtCore.QRect(120, 30, 51, 16))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.label_7.setFont(font)
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.label_8 = QtGui.QLabel(self.groupBox_3)
        self.label_8.setGeometry(QtCore.QRect(180, 30, 81, 20))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.label_8.setFont(font)
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.label_9 = QtGui.QLabel(self.groupBox_3)
        self.label_9.setGeometry(QtCore.QRect(270, 30, 51, 16))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.label_9.setFont(font)
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.vehicles_connection_status = QtGui.QListWidget(self.groupBox_3)
        self.vehicles_connection_status.setGeometry(QtCore.QRect(30, 50, 61, 131))
        self.vehicles_connection_status.setObjectName(_fromUtf8("vehicles_connection_status"))
        self.vehicles_armed_status = QtGui.QListWidget(self.groupBox_3)
        self.vehicles_armed_status.setGeometry(QtCore.QRect(110, 50, 61, 131))
        self.vehicles_armed_status.setObjectName(_fromUtf8("vehicles_armed_status"))
        self.vehicles_mode_status = QtGui.QListWidget(self.groupBox_3)
        self.vehicles_mode_status.setGeometry(QtCore.QRect(190, 50, 61, 131))
        self.vehicles_mode_status.setObjectName(_fromUtf8("vehicles_mode_status"))
        self.vehicles_battery_status = QtGui.QListWidget(self.groupBox_3)
        self.vehicles_battery_status.setGeometry(QtCore.QRect(270, 50, 61, 131))
        self.vehicles_battery_status.setObjectName(_fromUtf8("vehicles_battery_status"))
        self.groupBox_4 = QtGui.QGroupBox(Form)
        self.groupBox_4.setGeometry(QtCore.QRect(20, 500, 361, 181))
        self.groupBox_4.setObjectName(_fromUtf8("groupBox_4"))
        self.label_10 = QtGui.QLabel(self.groupBox_4)
        self.label_10.setGeometry(QtCore.QRect(70, 20, 101, 16))
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.cmd_vehicle_number = QtGui.QLineEdit(self.groupBox_4)
        self.cmd_vehicle_number.setGeometry(QtCore.QRect(180, 20, 113, 20))
        self.cmd_vehicle_number.setAlignment(QtCore.Qt.AlignCenter)
        self.cmd_vehicle_number.setObjectName(_fromUtf8("cmd_vehicle_number"))
        self.arm_button = QtGui.QPushButton(self.groupBox_4)
        self.arm_button.setGeometry(QtCore.QRect(20, 50, 51, 24))
        self.arm_button.setObjectName(_fromUtf8("arm_button"))
        self.disarm_button = QtGui.QPushButton(self.groupBox_4)
        self.disarm_button.setGeometry(QtCore.QRect(80, 50, 51, 24))
        self.disarm_button.setObjectName(_fromUtf8("disarm_button"))
        self.takoff_button = QtGui.QPushButton(self.groupBox_4)
        self.takoff_button.setGeometry(QtCore.QRect(150, 50, 51, 24))
        self.takoff_button.setObjectName(_fromUtf8("takoff_button"))
        self.land_button = QtGui.QPushButton(self.groupBox_4)
        self.land_button.setGeometry(QtCore.QRect(210, 50, 51, 24))
        self.land_button.setObjectName(_fromUtf8("land_button"))
        self.hold_button = QtGui.QPushButton(self.groupBox_4)
        self.hold_button.setGeometry(QtCore.QRect(270, 50, 51, 24))
        self.hold_button.setObjectName(_fromUtf8("hold_button"))
        self.shutdownOBC_button = QtGui.QPushButton(self.groupBox_4)
        self.shutdownOBC_button.setGeometry(QtCore.QRect(50, 80, 121, 24))
        self.shutdownOBC_button.setObjectName(_fromUtf8("shutdownOBC_button"))
        self.rebootOBC_button = QtGui.QPushButton(self.groupBox_4)
        self.rebootOBC_button.setGeometry(QtCore.QRect(180, 80, 121, 24))
        self.rebootOBC_button.setObjectName(_fromUtf8("rebootOBC_button"))
        self.setOrigin_button = QtGui.QPushButton(self.groupBox_4)
        self.setOrigin_button.setGeometry(QtCore.QRect(20, 110, 121, 24))
        self.setOrigin_button.setObjectName(_fromUtf8("setOrigin_button"))
        self.setEast_button = QtGui.QPushButton(self.groupBox_4)
        self.setEast_button.setGeometry(QtCore.QRect(20, 140, 121, 24))
        self.setEast_button.setObjectName(_fromUtf8("setEast_button"))
        self.origin_lat_edit = QtGui.QLineEdit(self.groupBox_4)
        self.origin_lat_edit.setGeometry(QtCore.QRect(160, 110, 91, 20))
        self.origin_lat_edit.setAlignment(QtCore.Qt.AlignCenter)
        self.origin_lat_edit.setObjectName(_fromUtf8("origin_lat_edit"))
        self.origin_long_edit = QtGui.QLineEdit(self.groupBox_4)
        self.origin_long_edit.setGeometry(QtCore.QRect(260, 110, 91, 20))
        self.origin_long_edit.setAlignment(QtCore.Qt.AlignCenter)
        self.origin_long_edit.setObjectName(_fromUtf8("origin_long_edit"))
        self.east_lat_edit = QtGui.QLineEdit(self.groupBox_4)
        self.east_lat_edit.setGeometry(QtCore.QRect(160, 140, 91, 20))
        self.east_lat_edit.setAlignment(QtCore.Qt.AlignCenter)
        self.east_lat_edit.setObjectName(_fromUtf8("east_lat_edit"))
        self.east_long_edit = QtGui.QLineEdit(self.groupBox_4)
        self.east_long_edit.setGeometry(QtCore.QRect(260, 140, 91, 20))
        self.east_long_edit.setAlignment(QtCore.Qt.AlignCenter)
        self.east_long_edit.setObjectName(_fromUtf8("east_long_edit"))
        self.groupBox_5 = QtGui.QGroupBox(Form)
        self.groupBox_5.setGeometry(QtCore.QRect(20, 690, 361, 80))
        self.groupBox_5.setObjectName(_fromUtf8("groupBox_5"))
        self.startMission_button = QtGui.QPushButton(self.groupBox_5)
        self.startMission_button.setGeometry(QtCore.QRect(80, 30, 79, 24))
        self.startMission_button.setObjectName(_fromUtf8("startMission_button"))
        self.stopMission_button = QtGui.QPushButton(self.groupBox_5)
        self.stopMission_button.setGeometry(QtCore.QRect(200, 30, 79, 24))
        self.stopMission_button.setObjectName(_fromUtf8("stopMission_button"))

        self.retranslateUi(Form)
        QtCore.QObject.connect(self.arm_button, QtCore.SIGNAL(_fromUtf8("clicked()")), Form.arm_slot)
        QtCore.QObject.connect(self.disarm_button, QtCore.SIGNAL(_fromUtf8("clicked()")), Form.disarm_slot)
        QtCore.QObject.connect(self.takoff_button, QtCore.SIGNAL(_fromUtf8("clicked()")), Form.takeoff_slot)
        QtCore.QObject.connect(self.land_button, QtCore.SIGNAL(_fromUtf8("clicked()")), Form.land_slot)
        QtCore.QObject.connect(self.hold_button, QtCore.SIGNAL(_fromUtf8("clicked()")), Form.hold_slot)
        QtCore.QObject.connect(self.shutdownOBC_button, QtCore.SIGNAL(_fromUtf8("clicked()")), Form.shutdownobc_slot)
        QtCore.QObject.connect(self.rebootOBC_button, QtCore.SIGNAL(_fromUtf8("clicked()")), Form.rebootobc_slot)
        QtCore.QObject.connect(self.setOrigin_button, QtCore.SIGNAL(_fromUtf8("clicked()")), Form.setorigin_slot)
        QtCore.QObject.connect(self.setEast_button, QtCore.SIGNAL(_fromUtf8("clicked()")), Form.seteast_slot)
        QtCore.QObject.connect(self.startMission_button, QtCore.SIGNAL(_fromUtf8("clicked()")), Form.startmission_slot)
        QtCore.QObject.connect(self.stopMission_button, QtCore.SIGNAL(_fromUtf8("clicked()")), Form.stopmission_slot)
        QtCore.QMetaObject.connectSlotsByName(Form)

        # ------------------ Added by user ------------------ 
        # ROS Publishers
        self.arm_pub        = rospy.Publisher('/arm_robot', Int32, queue_size=10)
        self.disarm_pub     = rospy.Publisher('/disarm_robot', Int32, queue_size=10)
        self.takeoff_pub    = rospy.Publisher('/takeoff_robot', Int32, queue_size=10)
        self.land_pub       = rospy.Publisher('/land_robot', Int32, queue_size=10)
        self.hold_pub       = rospy.Publisher('/hold_robot', Int32, queue_size=10)
        self.shutdown_pub   = rospy.Publisher('/shutdown_robot', Int32, queue_size=10)
        self.reboot_pub     = rospy.Publisher('/reboot_robot', Int32, queue_size=10)
        self.setorigin_pub  = rospy.Publisher('/setOrigin', Point, queue_size=10)
        self.seteast_pub    = rospy.Publisher('/setEast', Point, queue_size=10)
        self.start_pub      = rospy.Publisher('/start', Empty, queue_size=10)
        self.stop_pub       = rospy.Publisher('/stop', Empty, queue_size=10)

        # ROS Subscribers
        rospy.Subscriber('vehicles_states', MultiVehicleState, self.vehicles_states_Cb)

        # ROS Parameters
        # get nRobots
        self.nRobots = rospy.get_param("nRobots", 5)

        # initialize lists
        self.vehicles_connection_status.clear()
        self.vehicles_battery_status.clear()
        self.vehicles_mode_status.clear()
        self.vehicles_armed_status.clear()

        red = "#FF0000"
        for i in range(self.nRobots):
            bool_itme = QListWidgetItem("%i- False" %(i+1))
            bool_item.setBackground( QColor(red) )
            self.vehicles_connection_status.addItem(bool_item)

            self.vehicles_armed_status.addItem(bool_item)

            mode_item = QListWidgetItem("%i- Unknown" %(i+1))
            mode_item.setBackground( QColor(red) )
            self.vehicles_mode_status.addItem(mode_item)

            bat_item = QListWidgetItem("%i- 0.0" %(i+1))
            bat_item.setBackground( QColor(red) )
            self.vehicles_battery_status.addItem(bat_item)


    def vehicles_states_Cb(self, msg):
        # clear lists
        self.vehicles_connection_status.clear()
        self.vehicles_armed_status.clear()
        self.vehicles_mode_status.clear()
        self.vehicles_battery_status.clear()

        # Initialize overall system status flags
        sys_connection      = True
        sys_armed           = True
        sys_health          = True

        if len(msg.vehicles_states) == self.nRobots:

            # Define colors for bool flags
            red             = "#FF0000"
            green           = "#008000"

            for in in range(self.nRobots):

                sys_connection = sys_connection and msg.vehicles_states[i].connected
                sys_armed = sys_armed and msg.vehicles_states[i].armed
                sys_health = sys_health and (False if msg.vehicles_states[i].health > 17 else True)

                # connection status
                if msg.vehicles_states[i].connected:
                    bool_itme = QListWidgetItem("%i- True" %(i+1))
                    bool_item.setBackground( QColor(green) )
                    self.vehicles_connection_status.addItem(bool_item)
                else:
                    bool_itme = QListWidgetItem("%i- False" %(i+1))
                    bool_item.setBackground( QColor(red) )
                    self.vehicles_connection_status.addItem(bool_item)

                # armed status
                if msg.vehicles_states[i].armed:
                    bool_itme = QListWidgetItem("%i- True" %(i+1))
                    bool_item.setBackground( QColor(green) )
                    self.vehicles_armed_status.addItem(bool_item)
                else:
                    bool_itme = QListWidgetItem("%i- False" %(i+1))
                    bool_item.setBackground( QColor(red) )
                    self.vehicles_armed_status.addItem(bool_item)

                # flight mode
                mode_str = str(msg.vehicles_states[i].flight_mode)
                mode_item = QListWidgetItem("%i- "+mode_str %(i+1))
                self.vehicles_mode_status.addItem(mode_item)

                # battery level
                bat_str = str(msg.vehicles_states[i].battery)
                bat_item = QListWidgetItem("%i- "+bat_str %(i+1))
                self.vehicles_battery_status.addItem(bat_item)

            # Update over all system status

            if sys_connection:
                self.system_connection_status.setText('Connected')
                # TODO: Change background color to green!
            else:
                self.system_connection_status.setText('Not Connected')
                # TODO: Change background color to red!

            if sys_armed:
                self.system_armed_status.setText('Armed')
                # TODO: Change background color to greem!
            else:
                self.system_armed_status.setText('Not Armed')
                # TODO: Change background color to red!

            if sys_health:
                self.system_health_status.setText('Health OK')
                # TODO: Change background color to green!
            else:
                self.system_health_status.setText('Health Bad')
                # TODO: Change background color to red!
        else:
            rospy.logwarn("Number of vehicles in vehicles_status_msg is not equal to defined number of robots!")

    def arm_slot(self):
        # get vehicle number and convert to int
        vn = int(self.cmd_vehicle_number.text())

        # create msg object
        msg = Int32()
        #populate data
        msg.data = vn
        # publish
        self.arm_pub.publish(msg)
    
    def disarm_slot(self):
        # get vehicle number and convert to int
        vn = int(self.cmd_vehicle_number.text())

        # create msg object
        msg = Int32()
        #populate data
        msg.data = vn
        # publish
        self.disarm_pub.publish(msg)
    
    def takeoff_slot(self):
        # get vehicle number and convert to int
        vn = int(self.cmd_vehicle_number.text())

        # create msg object
        msg = Int32()
        #populate data
        msg.data = vn
        # publish
        self.takeoff_pub.publish(msg)

    def land_slot(self):
        # get vehicle number and convert to int
        vn = int(self.cmd_vehicle_number.text())

        # create msg object
        msg = Int32()
        #populate data
        msg.data = vn
        # publish
        self.land_pub.publish(msg)

    def hold_slot(self):
        # get vehicle number and convert to int
        vn = int(self.cmd_vehicle_number.text())

        # create msg object
        msg = Int32()
        #populate data
        msg.data = vn
        # publish
        self.hold_pub.publish(msg)

    def shutdownobc_slot(self):
        # get vehicle number and convert to int
        vn = int(self.cmd_vehicle_number.text())

        # create msg object
        msg = Int32()
        #populate data
        msg.data = vn
        # publish
        self.shutdown_pub.publish(msg)

    def rebootobc_slot(self):
        # get vehicle number and convert to int
        vn = int(self.cmd_vehicle_number.text())

        # create msg object
        msg = Int32()
        #populate data
        msg.data = vn
        # publish
        self.reboot_pub.publish(msg)

    def setorigin_slot(self):
        # get lat/long
        lat = float(self.origin_lat_edit.text())
        lon = float(self.origin_long_edit.text())

        # create msg object
        msg = Point()
        #populate data
        msg.x = lat
        msg.y = lon
        # publish
        self.setorigin_pub.publish(msg)

    def seteast_slot(self):
        # get lat/long
        lat = float(self.east_lat_edit.text())
        lon = float(self.east_long_edit.text())

        # create msg object
        msg = Point()
        #populate data
        msg.x = lat
        msg.y = lon
        # publish
        self.seteast_pub.publish(msg)

    def startmission_slot(self):
        msg = Empty()
        self.start_pub.publish(msg)

    def stopmission_slot(self):
        msg = Empty()
        self.stop_pub.publish(msg)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Form", None))
        self.label.setText(_translate("Form", "Formation Ground Station", None))
        self.groupBox.setTitle(_translate("Form", "Number of Vehicles", None))
        self.nRobotsEdit.setToolTip(_translate("Form", "<html><head/><body><p>Currently set by a ROS a paramter \'/nRobots\'</p></body></html>", None))
        self.nRobotsEdit.setText(_translate("Form", "5", None))
        self.groupBox_2.setTitle(_translate("Form", "Quick System Status", None))
        self.label_2.setText(_translate("Form", "The overall system status. Indvidual vehicles status is shown below", None))
        self.label_3.setText(_translate("Form", "Connection", None))
        self.label_4.setText(_translate("Form", "All ARMED?", None))
        self.label_5.setText(_translate("Form", "Health", None))
        self.system_connection_status.setToolTip(_translate("Form", "<html><head/><body><p>Currently set by a ROS a paramter \'/nRobots\'</p></body></html>", None))
        self.system_connection_status.setText(_translate("Form", "5", None))
        self.system_armed_status.setToolTip(_translate("Form", "<html><head/><body><p>Currently set by a ROS a paramter \'/nRobots\'</p></body></html>", None))
        self.system_armed_status.setText(_translate("Form", "5", None))
        self.system_health_status.setToolTip(_translate("Form", "<html><head/><body><p>Currently set by a ROS a paramter \'/nRobots\'</p></body></html>", None))
        self.system_health_status.setText(_translate("Form", "5", None))
        self.groupBox_3.setTitle(_translate("Form", "Vehicles Status", None))
        self.label_6.setText(_translate("Form", "Connection", None))
        self.label_7.setText(_translate("Form", "Armed?", None))
        self.label_8.setText(_translate("Form", "Flight Mode", None))
        self.label_9.setText(_translate("Form", "Battery", None))
        self.groupBox_4.setTitle(_translate("Form", "Commands", None))
        self.label_10.setText(_translate("Form", "Vehicle number", None))
        self.cmd_vehicle_number.setText(_translate("Form", "0", None))
        self.arm_button.setText(_translate("Form", "Arm", None))
        self.disarm_button.setText(_translate("Form", "Disarm", None))
        self.takoff_button.setText(_translate("Form", "Takeoff", None))
        self.land_button.setText(_translate("Form", "Land", None))
        self.hold_button.setText(_translate("Form", "Hold", None))
        self.shutdownOBC_button.setText(_translate("Form", "Shutdown OBC", None))
        self.rebootOBC_button.setText(_translate("Form", "Reboot OBC", None))
        self.setOrigin_button.setText(_translate("Form", "Set Origin Lat/Long", None))
        self.setEast_button.setText(_translate("Form", "Set East Lat/Long", None))
        self.origin_lat_edit.setText(_translate("Form", "21.12345", None))
        self.origin_long_edit.setText(_translate("Form", "49.12345", None))
        self.east_lat_edit.setText(_translate("Form", "21.12345", None))
        self.east_long_edit.setText(_translate("Form", "49.12345", None))
        self.groupBox_5.setTitle(_translate("Form", "Mission Commands", None))
        self.startMission_button.setText(_translate("Form", "START", None))
        self.stopMission_button.setText(_translate("Form", "STOP", None))


if __name__ == "__main__":
    import sys

    rospy.init_node('formation_gui')

    app = QtGui.QApplication(sys.argv)
    Form = QtGui.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

