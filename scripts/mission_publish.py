#!/usr/bin/env python

# mission_publish.py is connected to mission.py, and publishes the current mission (next GPS waypoint to go to) whenever the GPS data updates
# Every time that mission_publish.py publishes, the new GPS goal goes to mission_catcher.py, where it can be used by other programs
#
# It does this by publishing every time mission.py updates
# 		mission.py is connected to the GPS output - whenever the GPS data updates mission.py updates, and mission_publish.py publishes the result


# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import String

# Imports necessary sailbot code
from hardware import mission

def mission_publish():
    current_goal = mission.mission_goal.current_goal
    rospy.loginfo("mission_publish.py:Current goal = %s" %str(current_goal))

    # The current mission goal (in the format [x, y]) is publishes as a string, which makes it easy for ROS to publish and read
    mission_publisher.publish(str(current_goal))


rospy.sleep(10)
print 'init mission_publisher node'
current_node = rospy.init_node("mission_publisher", anonymous=True)
mission.init(current_node)
mission_publisher = rospy.Publisher("current_waypoint_goal", String, latch = True)

# This connects the mission_publish() function to mission.py
mission.mission_goal.set_callback(mission_publish)

rospy.spin()
