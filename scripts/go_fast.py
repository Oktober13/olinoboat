#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
from hardware import sensors
import rospy
from std_msgs.msg import String

def go_fast_cb(angle):
    
	go_fast_pub = rospy.Publisher('go_fast_topic', String)
    rospy.loginfo("Wind sensor sent %i" % (angle))

    upwind_cutoff = 50	    # Degrees
    downwind_cutoff = 10    # Degrees

    # This code makes it so that you can use pl.interp on relative_angles and go_or_not_go and it will return '0' if the given angle is directly downwind or in irons
    relative_angles = [0, upwind_cutoff - 1, upwind_cutoff, 180-downwind_cutoff-1, 180-downwind_cutoff, 180+downwind_cutoff, 180+downwind_cutoff+1, -upwind_cutoff-1, -upwind_cutoff, 0]
    relative_angles = relative_angles + angle
    for i in range(len(relative_angles))
    	relative_angles[i] = relative_angles[i] % 360
    go_or_not_go = [0, 0, 1, 1, 0, 0, 1, 1, 0]

    # Builds a list of 360 degree elements around the boat that say whether the boat can go in a given direction
    # If the 30th number in the list is 0, it means the boat shouldn't go 30 degrees to the right
    for i in range(360):
    	go_fast_heading[i] = pl.interp(i, relative_angles, go_or_not_go)

    go_fast_pub.publish(str(go_fast_heading))
    rospy.loginfo("go_fast_node suggested heading is:" + str(go_fast_heading)


current_node = rospy.init_node("go_fast_node",anonymous=True)
sensors.init(current_node)

sensors.wind_angle.set_callback(go_fast_cb)

rospy.spin()