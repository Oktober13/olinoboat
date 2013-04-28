#!/usr/bin/env python

import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16

class Offset():
	def __init__():
		self.xmin_pub = rospy.Publisher("xmin",UInt16)
		self.ymin_pub = rospy.Publisher("ymin",UInt16)
		self.zmin_pub = rospy.Publisher("zmin",UInt16)
		self.xmax_pub = rospy.Publisher("xmax",UInt16)
		self.ymax_pub = rospy.Publisher("ymax",UInt16)
		self.zmax_pub = rospy.Publisher("zmax",UInt16)
		self.pwm_offset_pub = rospy.Publisher("offset", UInt16)

	def pub_offset(self):
		lines = open("calibration.txt").read().splitlines()
		var_dic = {}
		for line in lines:
			variable, value = line.split(' ')
			var_dic[variable] = value
		if "xmin" in var_dic:
			self.xmin_pub.publish(UInt16(var_dic['xmin']))
		if "ymin" in var_dic:
			self.ymin_pub.publish(UInt16(var_dic['ymin']))
		if "zmin" in var_dic:
			self.zmin_pub.publish(UInt16(var_dic['zmin']))
		if "xmax" in var_dic:
			self.xmax_pub.publish(UInt16(var_dic['xmax']))
		if "ymax" in var_dic:
			self.ymax_pub.publish(UInt16(var_dic['ymax']))
		if "zmax" in var_dic:
			self.zmax_pub.publish(UInt16(var_dic['zmax']))
		if "pwm_offset" in var_dic:
			self.pwm_offset_pub.publish(UInt16(var_dic['pwm_offset']))
	


if __name__ == "__main__":
	rospy.sleep(10)
	o = Offset()
	o.pub_offset
	rospy.spin()
