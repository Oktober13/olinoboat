#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16


class datalistener():

    def __init__(self, offset = 0, node = 0):
        self.offset = offset
	self.wind_angle = 0
	self.leak = 0
	self.compass_angle = 0
	self.gps_pos = [0,0,0]

	#initialize node
        if node == 0:
            rospy.init_node('data_listener', anonymous=False)
	
	#set up subscribers to ("topic", DataType, callback_function)
        rospy.Subscriber("pwm_offset", UInt16, self.offset_callback)
        rospy.Subscriber("pwm_duration", UInt16, self.pwm_callback)
	rospy.Subscriber("leak", UInt16, self.water_callback)
        rospy.Subscriber("heading", UInt16, self.compass_callback)
        rospy.Subscriber("GPS_output", UInt16, self.gps_callback)

    ## define callback functions
    def offset_callback(self, data):
        self.offset = data.data

    def pwm_callback(self, data):
        phigh = data.data
        self.wind_angle = (360.*(phigh - self.offset)/1024.)%360

    def water_callback(self, data):
        if data.data > 1:
            self.leak = 1
	else: self.leak = 0	#May need to comment this out, reset self.leak manually in the event of a leak.

    def compass_callback(self, data):
        self.compass_angle = data.data

    def gps_callback(self, data):
        self.gps_pos = data.data
 
 
if __name__ == '__main__':
    listener = datalistener()
    rospy.spin()
