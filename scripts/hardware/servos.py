#!/usr/bin/env python

# servos.py contains a class that needs to be initialized by another piece of code to use
# More information about python 'classes' here: http://www.tutorialspoint.com/python/python_classes_objects.htm
#
# servos.py is initialized by every piece of code that needs to control the sail or rudder
#       04/14/2013 - Files that use servos.py
#               maintain_fast_sail_angle.py -- point_boat_at_target.py
#
# See examples in every piece of code that imports sensors.py

# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16

sail = None
rudder = None

class Servo: 
    def __init__(self, name, node=0):
        #initialize node if not being initialized by think code
        if node == 0:
            rospy.init_node('data_listener', anonymous=False)

        self.name = name    
        self.__publisher = rospy.Publisher(self.name, UInt16)
        #self.set_position(0)
        self.current_position = 0
        self.servomin = 0
        self.servomax = 180
	self.degmin = 45.	#set the bounds of the signal you send to the servo (45 to 135 for the rudder(?) , ?? to ?? for the sail)
	self.degmax = 135
	self.get_offset()

    def get_offset(self):
        lines = open("../calibration.txt").read().splitlines()
        var_dic = {}
        for line in lines:
            variable, value = line.split(' ')
            var_dic[variable] = value
        if self.name == rudder:
            if "ruddermin" in var_dic:
                self.servomin = var_dic['ruddermin']
            if "ruddermax" in var_dic:
                self.servomax = var_dic['ruddermax']
        if self.name == sail:
            if "sailmin" in var_dic:
                self.servomin = var_dic['sailmin']
            if "sailmax" in var_dic:
                self.servomax = var_dic['sailmax']
	

    def set_position(self,degrees):
        s = type(degrees)
        #rospy.loginfo("servos.py:  The degrees variable is %i, type is %s" %(degrees, s))
        #self.__publisher.publish(UInt16(degrees))
	servo_cmd = self.angle_to_command(degrees)
        #rospy.loginfo("servos.py: %s moved to %i degrees" % (self.name, servo_cmd))
        self.current_position = servo_cmd

    def angle_to_command(self,degrees):
        cmd = float(degrees-self.degmin) * ( (self.servomax-self.servomin)/(self.degmax-self.degmin) ) + self.servomin
        print cmd
        return cmd

def init(node):
    global sail 
    global rudder
    
    sail = Servo("sail", node)
    rudder = Servo("rudder", node)
