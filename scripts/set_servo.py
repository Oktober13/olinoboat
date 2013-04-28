#!/usr/bin/env python

# set_offset.py is used for setting the servo performance to correspond to the real world.
# This is necessary because the 'zero' point of the servo is not the same on the rudder or sail.


# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import String
#from std_msgs.msg import Uint16
import teleop

def ask_user():
	print "Please follow the instructions in the callibration section of the user manual"
	(rudder, sail) = t.get_user_input()
	ruddermin = rudder
	(rudder, sail) = t.get_user_input()
	rudder0 = rudder
	(rudder, sail) = t.get_user_input()
	ruddermax = rudder
	(rudder, sail) = t.get_user_input()
	sailmin = sail
	(rudder, sail) = t.get_user_input()
	sailmax = sail
	print "rudder (min, 0, max): (%f, %f, %f)" %(ruddermin, rudder0, ruddermax)
	print "sail (min, max): (%f, %f)" %(sailmin, sailmax)
	
	servo_offsets = str([ruddermin, rudder0, ruddermax, sailmin, sailmax])
	servo_offset_publisher = rospy.Publisher("servo_offsets", String, latch = True)
	servo_offset_publisher.publish(servo_offsets)
	return servo_offsets
	"""rudder_min_publisher = rospy.Publisher("rudder_min", UInt16, latch = True)
	rudder_min_publisher.publish(ruddermin)
	rudder_0_publisher = rospy.Publisher("rudder_0", UInt16, latch = True)
	rudder_min_publisher.publish(rudder0)
	rudder_max_publisher = rospy.Publisher("rudder_max", UInt16, latch = True)
	rudder_max_publisher.publish(ruddermax)
	sail_min_publisher = rospy.Publisher("sail_min", UInt16, latch = True)
	sail_min_publisher.publish(sailmin)
	sail_max_publisher = rospy.Publisher("sail_max", UInt16, latch = True)
	sail_max_publisher.publish(sailmax)"""
	

if __name__ == "__main__":
	#rospy.init_node('servo_offset_setter')
	rospy.sleep(7)
	t = teleop.teleop()
	ask_user()
	rospy.sleep(2)

    

