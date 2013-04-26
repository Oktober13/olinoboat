#!/usr/bin/env python

# think.py creates a loop that calls other code
#
# Dependent on : go_fast.py, go_short.py
#		think.py takes the outputs of go_fast and go_short, multiplies them together, and then chooses the strongest element as the heading to go to
#		See code explanation documentation for a more in-depth explanation of the arbiter

#hello Eric
# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import Int16
import time
import matplotlib.pyplot as plt
from math import pi

# Imports necessary sailbot code
import go_fast
import go_short

# If you set this to True, think.py will try to make a graph of where it wants you to go
plot_heading = True

think_node = rospy.init_node("think")

go_fast.init(think_node)
go_short.init(think_node)

think_command_pub = rospy.Publisher('desired_heading', Int16)

# This is the 'think loop', which redecides which direction to go every cycle
while not rospy.is_shutdown():
	time.sleep(5)
	fast_behavior = go_fast.suggested_heading
	short_behavior = go_short.suggested_heading
        desired_behavior = [fast*short for fast, short in zip(fast_behavior, short_behavior)]
	desired_heading = desired_behavior.index(max(desired_behavior))
	if desired_heading > 180: desired_heading -= 360
	
	think_command_pub.publish(Int16(desired_heading))
	rospy.loginfo("think.py chose this heading: %f degrees" %desired_heading)

	if plot_heading == True:
		x = [i*pi/180 for i in range(360)]
		plt1 = plt.subplot(1, 3, 1, projection='polar')
		plt1.set_theta_zero_location('N')
		plt1.set_theta_direction(-1)
		plt.polar(x, desired_behavior)
		plt.title('think is a combination of go_fast and go_short')

		plt2 = plt.subplot(1, 3, 2, projection='polar')
		plt2.set_theta_zero_location('N')
		plt2.set_theta_direction(-1)
		plt.polar(x, fast_behavior)
		plt.title('go_fast is 0 directly upwind and downwind')
		plt.polar(2, 1.1)

		plt3 = plt.subplot(1, 3, 3, projection='polar')
		plt3.set_theta_zero_location('N')
		plt3.set_theta_direction(-1)
		plt.polar(x, short_behavior)
		plt.title('go_short wants to go directly at the next waypoint')

		plt.show()
