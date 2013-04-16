#!/usr/bin/env python

# think.py creates a loop that calls other code
#
# Dependent on : go_fast.py, go_short.py
#		think.py takes the outputs of go_fast and go_short, multiplies them together, and then chooses the strongest element as the heading to go to
#		See documentation for a more in-depth explanation of the arbiter


# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import Int16
import time
import matplotlib.pyplot as plt
from math import pi

# Imports necessary functions
import go_fast
import go_short

# If you set this to True, think.py will try to make a graph of where it wants you to go
plot_heading = True

think_node = rospy.init_node("think")

go_fast.init(think_node)
go_short.init(think_node)

think_command_pub = rospy.Publisher('desired_heading', Int16)

if plot_heading == True:
	x = [i*pi/180 for i in range(360)]

# This is the 'think loop', which redecides which direction to go every cycle
while not rospy.is_shutdown():
	time.sleep(5)
	fast_behavior = go_fast.suggested_heading
	short_behavior = go_short.suggested_heading
        desired_behavior = [fast*short for fast, short in zip(fast_behavior, short_behavior)]
	desired_heading = desired_behavior.index(max(desired_behavior))
	if desired_heading > 180: desired_heading -= 360
	
	think_command_pub.publish(Int16(desired_heading)))
	rospy.loginfo("think.py chose this heading: %f degrees" %desired_heading)

	if plot_heading == True:
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
