#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
import go_fast
import time

think_node = rospy.init_node("think")

go_fast.init(think_node)

while not rospy.is_shutdown():
	time.sleep(5)
	rospy.loginfo("go_fast_node suggested heading is:")
	fast_behavior = go_fast.suggested_heading
	short_behavior = go_short.suggested_heading
        desired_behavior = [fast*short for fast, short in zip(fast_behavior, short_behavior)]
	desired_heading = max(desired_behavior)
	print desired_heading
