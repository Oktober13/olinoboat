#!/usr/bin/env python
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
        self.set_position(0)
        self.current_position = 0

    def set_position(self,degrees,):
        self.__publisher.publish(degrees)
        rospy.loginfo("servos.py: %s moved to %i degrees" % (self.name, degrees))
        self.current_position = degrees
        

def init(node):
    global sail 
    global rudder
    sail = Servo("sail", node)
    rudder = Servo("rudder", node)