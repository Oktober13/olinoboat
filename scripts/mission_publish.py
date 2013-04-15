#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from hardware import mission
from std_msgs.msg import String

def mission_publish():
    current_goal = mission.mission_goal.current_goal
    rospy.loginfo("mission_publish.py:Current goal = %s" %str(current_goal))

    mission_publisher = rospy.Publisher("current_waypoint_goal", String, latch = True)
    mission_publisher.publish(str(current_goal))

current_node = rospy.init_node("mission_publisher", anonymous=True)
mission.init(current_node)
mission.mission_goal.set_callback(mission_publish)

rospy.spin()