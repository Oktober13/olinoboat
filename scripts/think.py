#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
import go_fast
import go_short
import time
import matplotlib.pyplot as plt

think_node = rospy.init_node("think")

go_fast.init(think_node)
go_short.init(think_node)

x = [i*3.1415926535/180 for i in range(360)]


while not rospy.is_shutdown():
	time.sleep(5)
	fast_behavior = go_fast.suggested_heading
	short_behavior = go_short.suggested_heading
        desired_behavior = [fast*short for fast, short in zip(fast_behavior, short_behavior)]
	rospy.loginfo("think.py suggested heading is:")
	desired_heading = max(desired_behavior)
	print desired_heading
	fig = plt.figure()
	plt1 = plt.subplot(1, 1, 1, projection='polar')
	plt1.set_theta_zero_location('N')
	plt1.set_theta_direction(-1)
	plt.polar(x, desired_behavior)
	plt.show()
