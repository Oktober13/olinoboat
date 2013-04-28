#!/usr/bin/env python

# think.py creates a loop that calls other code
#
# Dependent on : go_fast.py, go_short.py
#		think.py takes the outputs of go_fast and go_short, multiplies them together, and then chooses the strongest element as the heading to go to
#		See code explanation documentation for a more in-depth explanation of the arbiter

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

rospy.sleep(10)
print 'init think node'
think_node = rospy.init_node("think")

go_fast.init(think_node)
go_short.init(think_node)

think_command_pub = rospy.Publisher('desired_heading', Int16)

# This is the 'think loop', which redecides which direction to go every cycle
while not rospy.is_shutdown():
	time.sleep(1)
	fast_behavior = go_fast.suggested_heading
	short_behavior = go_short.suggested_heading
        desired_behavior = [fast*short for fast, short in zip(fast_behavior, short_behavior)]
	desired_heading = desired_behavior.index(max(desired_behavior))
	if desired_heading > 180: desired_heading -= 360
	
	think_command_pub.publish(Int16(desired_heading))
	rospy.loginfo("think.py chose this heading: %f degrees" %desired_heading)
