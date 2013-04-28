#!/usr/bin/env python

# set_offset.py is used for setting the wind angle to a new zero
# This is necessary because the 'zero' point of the encoder is not necessarily straight forward on the boat
#		You run set_offset.py when the wind sensor is pointing directly forward, and the think code will treat that as the new 'zero' for the wind sensor

# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import String

class compass_maxmin():
	def __init__(self):
		self.xmin = 2047
		self.ymin = 2047
		self.zmin = 2047
		self.xmax = -2048
		self.ymax = -2048
		self.zmax = -2048
		self.xmin_called_back = 0
		self.ymin_called_back = 0
		self.zmin_called_back = 0
		self.xmax_called_back = 0
		self.ymax_called_back = 0
		self.zmax_called_back = 0

	def xmin_callback(self, data):
		self.xmin = data.data
		self.xmin_called_back = 1
	def ymin_callback(self, data):
		self.ymin = data.data
		self.ymin_called_back = 1
	def zmin_callback(self, data):
		self.zmin = data.data
		self.zmin_called_back = 1
	def xmax_callback(self, data):
		self.xmax = data.data
		self.xmax_called_back = 1
	def ymax_callback(self, data):
		self.ymax = data.data
		self.ymax_called_back = 1
	def zmax_callback(self, data):
		self.zmax = data.data
		self.zmax_called_back = 1
	
	


if __name__ == "__main__":
	rospy.init_node('encoder_offset_setter')
	cmm = compass_maxmin()
	rospy.sleep(5)

	rospy.Subscriber("xmin", UInt16, cmm.xmin_callback)
	rospy.Subscriber("ymin", UInt16, cmm.ymin_callback)
	rospy.Subscriber("zmin", UInt16, cmm.zmin_callback)
	rospy.Subscriber("xmax", UInt16, cmm.xmax_callback)
	rospy.Subscriber("ymax", UInt16, cmm.ymax_callback)
	rospy.Subscriber("zmax", UInt16, cmm.zmax_callback)

	while cmm.xmin_called_back + cmm.ymin_called_back + cmm.zmin_called_back + cmm.xmax_called_back + cmm.ymax_called_back + cmm.zmax_called_back != 6:
		rospy.sleep(1)
	compassmaxmin = str([cmm.xmin, cmm.ymin, cmm.zmin, cmm.xmax, cmm.ymax, cmm.zmax])
	compass_offset_publisher = rospy.Publisher("compass_offsets", String, latch = True)
	compass_offset_publisher.publish(compassmaxmin)
	rospy.spin()

	

