#!/usr/bin/env python

# set_offset.py is used for setting the servo performance to correspond to the real world.
# This is necessary because the 'zero' point of the servo is not the same on the rudder or sail.


# Imports necessary libraries
import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
from std_msgs.msg import UInt16
import teleop

def ask_user():
	print "Please follow the following instructions."
	print "turn the rudder servo to -45 degrees using the right and left arrow keys."
	(rudder, sail) = t.get_user_input()
	ruddermin = rudder
	print "turn the rudder servo to 0 degrees using the right and left arrow keys."
	(rudder, sail) = t.get_user_input()
	rudder0 = rudder
	print "turn the rudder servo to 45 degrees using the right and left arrow keys."
	(rudder, sail) = t.get_user_input()
	ruddermax = rudder
	print "Pull the sail all the way in using the up and down arrow keys."
	(rudder, sail) = t.get_user_input()
	sailmin = sail
	print "Let the sail all the way out using the up and down arrow keys."
	(rudder, sail) = t.get_user_input()
	sailmax = sail
	print "rudder (min, 0, max): (%f, %f, %f)" %(ruddermin, rudder0, ruddermax)
	print "sail (min, max): (%f, %f)" %(sailmin, sailmax)
	

if __name__ == "__main__":
	#rospy.init_node('servo_offset_setter')
	t = teleop.teleop()
	ask_user()
	rospy.sleep(2)

    

