#!/usr/bin/env python

import os
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import String
import set_compass
import set_servo
import set_offset

class calibration_writer():
	def __init__(self, node):
		self.node = node
		self.var_dic= {'pwm_offset':0, 'ruddermin':0, 'rudder0':90, 'ruddermax':180, 'sailmin':0, 'sailmax':90, 'xmin':0, 'xmax': 255, 'ymin':0, 'ymax': 255, 'zmin':0, 'zmax': 255}
		self.servo_called = 0
		self.compass_called = 0

	def write_wrapper(self):
		first = 1
		for variable in self.var_dic:
			if first == 1:
				self.write_to_file(variable, self.var_dic[variable], 'w')
				first = 0
			self.write_to_file(variable, self.var_dic[variable], 'a')


	def write_to_file(self, variable, value, mode):
		if os.path.isfile("calibration.txt"):
			f = open("calibration.txt",mode)
			f.write("%s %s\n"%(variable, value))
			f.close()
		else:
			print 'create a file named calibration.txt'

	def pwm_callback(self, data):
		self.var_dic['pwm_offset'] = data.data

	def compass_callback(self, data):	
		compass_offset = data.data.split(', ')
		self.var_dic['xmin'] = compass_offset[0].lstrip('[')
		self.var_dic['xmax'] = compass_offset[1]
		self.var_dic['ymin'] = compass_offset[2]
		self.var_dic['ymax'] = compass_offset[3]
		self.var_dic['zmin'] = compass_offset[4]
		self.var_dic['zmax'] = compass_offset[5].rstrip(']')
		self.compass_called = 1	

	def servo_callback(self, data):
		servo_offsets = data.data.split(', ')
		self.var_dic['ruddermin'] = servo_offsets[0].lstrip('[')
		self.var_dic['rudder0'] = servo_offsets[1]
		self.var_dic['ruddermax'] = servo_offsets[2]
		self.var_dic['sailmin'] = servo_offsets[3]
		self.var_dic['sailmax'] = servo_offsets[4].rstrip(']')
		self.servo_called = 1	

	

if __name__ == "__main__":
	node = rospy.init_node('calibration_node')
	#global var_dic 
	cw = calibration_writer(node)

	rospy.Subscriber("pwm_duration", UInt16, cw.pwm_callback)
	rospy.Subscriber("compass_offsets", String, cw.compass_callback)
	rospy.Subscriber("servo_offsets", String, cw.servo_callback)

	while cw.servo_called + cw.compass_called != 2:
		rospy.sleep(1)
	cw.write_wrapper()
	print "Written to file"
	rospy.spin()

