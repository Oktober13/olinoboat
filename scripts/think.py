#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16
import go_fast
import go_short
import time
import matplotlib.pyplot as plt

think_node = rospy.init_node("think")

go_fast.init(think_node)
go_short.init(think_node)

think_command_pub = rospy.Publisher('desired_heading', UInt16)

x = [i*3.1415926535/180 for i in range(360)]


while not rospy.is_shutdown():
	time.sleep(5)
	fast_behavior = go_fast.suggested_heading
	short_behavior = go_short.suggested_heading
        desired_behavior = [fast*short for fast, short in zip(fast_behavior, short_behavior)]
	desired_heading = desired_behavior.index(max(desired_behavior))
	if desired_heading > 180: desired_heading -= 360
	think_command_pub.publish(desired_heading)
	rospy.loginfo("think.py suggested heading is: %f" %desired_heading)

#	fig = plt.figure()
	
	plt1 = plt.subplot(1, 3, 1, projection='polar')
	plt1.set_theta_zero_location('N')
	plt1.set_theta_direction(-1)

	plt2 = plt.subplot(1, 3, 2, projection='polar')
	plt2.set_theta_zero_location('N')
	plt2.set_theta_direction(-1)

	plt3 = plt.subplot(1, 3, 3, projection='polar')
	plt3.set_theta_zero_location('N')
	plt3.set_theta_direction(-1)

	plt.subplot(1, 3, 2)
	plt.polar(x, fast_behavior)
	plt.subplot(1, 3, 1)
	plt.polar(x, desired_behavior)
	plt.subplot(1, 3, 3)
	plt.polar(x, short_behavior)

	plt.show()

