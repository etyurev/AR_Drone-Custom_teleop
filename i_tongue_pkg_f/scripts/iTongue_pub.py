#!/usr/bin/env python
# Author: Mathias Poulsen
# Edited from Mostafa Mohammadi
# Date: 19 mar 2019


import rospy

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import Bool
import numpy
import math
import sys
import serial
from PyQt4 import QtGui, QtCore

tongue_mode = 1  #1 for tongue_mode -1 for hand mode
y_upper_lim = 100

class serial_handle(QtCore.QObject):

    serial_line = QtCore.pyqtSignal()

    def __init__(self, serialObj):
        super(serial_handle, self).__init__()
        self.reading = False
        self.serial = serialObj

    def read_it(self):
        while self.reading:
            serialData = self.serial.readline().decode('ascii')
            #print(serialData)
            if serialData[0]=='S':
                self.sensors = []
                for i in range(2):
                    #accessing element 4,5 and 7,8 as the x,y position
                    self.sensors.append(int(serialData[3*i+4:3*i+6],16))

                self.serial_line.emit()


class Window(QtGui.QMainWindow):

    def __init__(self):
        super(Window, self).__init__()
        self.setGeometry(400, 300, 350, 150)
        self.setWindowTitle('ITCI Connection')

        # Global variables
        self.serialObj = serial.Serial()
        self.serialObj.baudrate = 115200
        self.serialObj.timeout = 1
        self.serial_handler = serial_handle(self.serialObj)
        self.thread = QtCore.QThread()
        self.serial_handler.moveToThread(self.thread)
        self.thread.started.connect(self.serial_handler.read_it)
        self.serial_handler.serial_line.connect(self.read_serial)
        #ROS publishers
        self.pub = rospy.Publisher('AU_position', Pose2D, queue_size=10)
        self.pose = Pose2D()
        self.pubVel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pubTake = rospy.Publisher('/ardrone/takeoff', Empty, queue_size = 1)
        self.pubLand = rospy.Publisher('/ardrone/land', Empty, queue_size = 1)
        self.pubEmer = rospy.Publisher('/ardrone/reset', Empty, queue_size = 1)
        self.pubHover = rospy.Publisher('/hover_mode', Bool, queue_size = 1)
        self.subCollision = rospy.Subscriber("/Collision",Bool, self.collision_callback)
        self.empty_msg = Empty()
        self.H_count = 0
        self.T_count = 0
        self.A_count = 0
        self.activated = False
        self.home()

    def home(self):
        # Initialization
        font = QtGui.QFont()
        font.setPointSize(12)

        # The Connect group
        self.groupBoxConnect = QtGui.QGroupBox( 'Connect', self)
        self.groupBoxConnect.setGeometry(QtCore.QRect(10, 10, 300, 100))

        self.gridConnect = QtGui.QGridLayout(self.groupBoxConnect)
        self.gridConnect.setHorizontalSpacing(20)

        self.labelCOM = QtGui.QLabel("Port Name", self.groupBoxConnect)
        self.labelCOM.setGeometry(QtCore.QRect(10, 10, 150, 60))
        self.labelCOM.setFont(font)
        self.gridConnect.addWidget(self.labelCOM,0,0)

        self.comboBoxPort = QtGui.QComboBox(self)
        self.comboBoxPort.addItem('/dev/rfcomm0')
        self.comboBoxPort.addItem('/dev/rfcomm1')
        self.comboBoxPort.addItem('/dev/rfcomm2')
        self.comboBoxPort.addItem('/dev/rfcomm3')
        self.comboBoxPort.addItem('/dev/rfcomm4')
        self.comboBoxPort.addItem('/dev/rfcomm5')
        self.comboBoxPort.addItem('/dev/rfcomm6')
        self.comboBoxPort.addItem('/dev/rfcomm7')
        self.comboBoxPort.addItem('/dev/rfcomm8')

        self.gridConnect.addWidget(self.comboBoxPort,1,0)

        self.pushButtonPort = QtGui.QPushButton(self.groupBoxConnect)
        self.pushButtonPort.setText('Connect')
        self.pushButtonPort.clicked.connect(self.connect_serial)
        self.gridConnect.addWidget(self.pushButtonPort,1,1)

        self.show()

    def connect_serial(self):
        if not self.serialObj.is_open:
            #try:
                self.serialObj.port = self.comboBoxPort.currentText()
                self.serialObj.open()
                print(self.serialObj.is_open)
                self.pushButtonPort.setText('Disconnect')
                self.serial_handler.reading = True
                self.thread.start()
                #print('Averaging sample is '+self.lineEditAveraging.text())
            #except:
            #    print('This Serial Port Is Not Available')
        else:
            self.pushButtonPort.setText('Connect')
            self.serial_handler.reading = False
            self.thread.terminate()
            self.serialObj.close()

    def collision_callback(self, bool_data):
        if bool_data.data:
            self.activated = True
        elif not bool_data.data:
            self.activated = False


    def move_drone(self, key, velocity):
        #print(key)
        TA_var = 9 # 1 step corresponds to 0.033333 s (30 Hz)
        vel_cmd = Twist()
        vel = 0.3
        if key == 'F':
            vel_cmd.linear.x = velocity
        elif key == 'R':
            vel_cmd.angular.z = -1*tongue_mode*0.6#1.5*vel#ocity[1]
        elif key == 'L':
            vel_cmd.angular.z = tongue_mode*0.6#*vel#ocity[1]
        elif key == 'S':
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = 0
            vel_cmd.linear.z = 0
            vel_cmd.linear.y = 0
            H_count=0
        elif key == 'U':
            vel_cmd.linear.z =  0.5
        elif key == 'D':
            vel_cmd.linear.z = -0.5
        elif key == 'A':
            self.A_count = self.A_count + 1
            if self.A_count > TA_var:
                self.A_count = 0
                self.pubLand.publish(self.empty_msg)
                print('Landing')
        elif key == 'T':
            self.T_count = self.T_count + 1
            if self.T_count > TA_var:
                self.T_count = 0
                self.pubTake.publish(self.empty_msg)
                print('Take Off')
        elif key == 'RF':
            vel_cmd.linear.y=-1*tongue_mode*0.5*vel#ocity[0]
            vel_cmd.linear.x=0.5*vel
            #vel_cmd.angular.z=-velocity[1]
        elif key == 'LF':
            vel_cmd.linear.y=tongue_mode*0.5*vel#ocity[0]
            vel_cmd.linear.x=0.5*vel
            #vel_cmd.angular.z=velocity[1]
        elif key == 'HL':
            vel_cmd.linear.y=tongue_mode*velocity
        elif key == 'HR':
            vel_cmd.linear.y=-1*tongue_mode*velocity
        #below only used for hovering until self.pubvel.publish
        elif key == 'H':
            self.H_count = self.H_count + 1
        if key != 'H':
            self.H_count = 0
        if key != 'A':
            self.A_count = 0
        if key != 'T':
            self.T_count = 0

        if self.H_count > 20:
            if not self.activated:
                print('Emergency Stop')
                self.pubEmer.publish(self.empty_msg)
                self.activated = True
                self.H_count = 0
            elif self.activated:
                #print('deactivate hover')
                self.pubEmer.publish(self.empty_msg)
                self.activated = False
                self.H_count = 0

        if not self.activated:
            self.pubVel.publish(vel_cmd)

    def read_serial(self):

        position=self.serial_handler.sensors
        self.pose.x = position[0]
        self.pose.y = position[1]
        #publishing 2D pose
        self.pub.publish(self.pose)

        #print('x',position[0])
        #print('y',position[1])
        #which_direction='S'
        #Change left and right
        which_direction=self.which_direction2(position)
        value=[]
        #Combination to stop.'when not activated for some time.' And
        # and not stop when when just activated for little time.
        print('which direction: ', which_direction)
        if which_direction == 'F':
            value=(position[1]-177)/49.0
            value=self.value_exponential(value)
            print('value', value)
        elif which_direction == 'HL':
            value=(position[0]-60)/67.0
            value=self.value_exponential(value)
            print('value', value)
        elif which_direction == 'HR':
            value=(196-position[0])/67.0
            value=self.value_exponential(value)
            print('value:', value)

            #print('value',value)
        self.move_drone(which_direction,value)

    #new dividings
    def which_direction2(self,position):
        #checks left right
        which_direction='S'
        if position[0] != 128 and position[1] > 128:
            #checking center
            if position[0] <= 60 or position[0] >= 196:
                if position[1] > 177:
                    which_direction = 'F'
                else:
                    which_direction = 'H'
            elif position[0] > 60 and position[0] <= 127:
                if position[1] < 154:
                    which_direction = 'L'
                elif position[1] > 201:
                    which_direction = 'LF'
                else:
                    which_direction = 'HL'
            elif position[0] >= 129 and position[0] < 196:
                if position[1] < 154:
                    which_direction = 'R'
                elif position[1] > 201:
                    which_direction = 'RF'
                else:
                    which_direction = 'HR'
        elif position[1] < 128:
            #top left corner
            if position[1] > y_upper_lim and position[0] > 128:
                #print('TakeOff')
                which_direction='T'
            elif position[1] > y_upper_lim and position[0] < 128:
                #print('Land')
                which_direction='A'
            elif position[1] < y_upper_lim and position[0] > 128:
                print('U')
                which_direction='U'
            elif position[1] < y_upper_lim and position[0] < 128:
                print('D')
                which_direction='D'
        else:
            print('not activated')
            which_direction='S'
        #print('x ',position[0])
        #print('y ',position[1])
        return which_direction

    def vector_projection(self,vector,quadrant):
        #Finds vector projection. Returns a value with normalized values
        #for the. [0] is forward. [1] is rotational.
        value=[]
        if quadrant == 'F':
            value.append(vector[1]/81.0)
            value.append(0)
        elif quadrant == 'L' or quadrant == 'R':
            value.append(0)
            value.append(vector[0]/128.0)
        elif quadrant == 'RF' or quadrant == 'LF':
            value.append(vector[1]/81.0)
            value.append(vector[0]/128.0)
        return value

    def value_exponential(self,value):
        #Finds a value on a exponential scale with base exp
        exp=7
        speed=0.3
        temp=math.pow(exp,value)/exp*speed
        #temp.append(math.pow(exp,value)/exp*speed)
        #temp.append(math.pow(exp,value[1])/exp*speed)
        return temp

    def find_vector(self,position):
        vector=[]
        if position[0]>128:
            vector.append(256-position[0])
        else:
            vector.append(position[0])
        vector.append(abs(position[1]-145))
        return vector


if __name__ == '__main__':
    try:
        rospy.init_node('iTongue_pub')
        app = QtGui.QApplication(sys.argv)
        GUI = Window()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
