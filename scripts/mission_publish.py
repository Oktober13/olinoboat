#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from hardware import mission

def mission_publish():
	current_goal = mission.mission_goal.current_goal
	# Then publishes this data? Not sure if we need this file. Will think about it

current_node = rospy.init_node("mission_publish",anonymous=True)

mission.init(current_node)
mission.mission_goal.set_callback(mission_publish)

rospy.spin()