#!/usr/bin/env python

import os
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import String

def write_wrapper(var_dic):
	first = 1
	for variable in var_dic:
		if first == 1:
			write_to_file(variable, var_dic[variable], 'w')
			first = 0
		else: 
			write_to_file(variable, var_dic[variable], 'a')


def write_to_file(variable, value, mode):
	if os.path.isfile("calibration.txt"):
		f = open("calibration.txt",mode)
		f.write("%s %f\n"%(variable, value))
		f.close()
	else:
		print 'create a file named calibration.txt'

def pwm_callback(data):
	var_dic[pwm_offset] = data.data

def compass_callback(data):
	compass_offset = split(data.data, ',')
	var_dic[xmin] = compass_offset[0]
	var_dic[xmax] = compass_offset[1]
	var_dic[ymin] = compass_offset[2]
	var_dic[ymax] = compass_offset[3]
	var_dic[zmin] = compass_offset[4]
	var_dic[zmax] = compass_offset[5]

def servo_callback(data):
	servo_offsets = split(data.data, ',')
	var_dic[ruddermin] = servo_offsets[0]
	var_dic[rudder0] = servo_offsets[1]
	var_dic[ruddermax] = servo_offsets[2]
	var_dic[sailmin] = servo_offsets[3]
	var_dic[sailmax] = servo_offsets[4]

	

if __name__ == "__main__":
	rospy.init_node('calibration_node')
	#global var_dic 
	var_dic= {'pwm_offset':0, 'ruddermin':0, 'rudder0':90, 'ruddermax':180, 'sailmin':0, 'sailmax':180, 'xmin':0, 'xmax': 255, 'ymin':0, 'ymax': 255, 'zmin':0, 'zmax': 255,}
	rospy.Subscriber("pwm_duration", UInt16, pwm_callback)
	rospy.Subscriber("heading", String, compass_callback)
	rospy.Subscriber("servo_offsets", String, servo_callback)
	write_wrapper(var_dic)
	rospy.sleep(2)

