#!/usr/bin/env python

# point_boat_at_target.py changes the rudder position every time the compass heading updates
# It is dependent on sensors.py for the compass output, and think.py for the direction the boat should be heading
# This code outputs to servos.py
#
# At a high level, as of 04/14/2013, this code tries to use the rudder to point the boat in the intended direction using PID control (more explanation below)

# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
import pylab as pl
from std_msgs.msg import UInt16

# This function using PID control to get the boat pointed at the correct compass heading
# This function updates every time the compass heading updates
# You can find more information about pid control here: http://www.csimn.com/CSI_pages/PIDforDummies.html
# If you read the paper:
#		The Setpoint of the PID control is the command 'boat_heading.setPoint(desired_compass_heading)'
#		The Process Variable is the compass data, seen in the command 'boat_heading.update(compass_heading)'
#		The Controller Output, in this case, is called pid_command, and we use that to set our rudder angle
def auto_trim_sails(data):
	wind_angle = data.data
	# rospy.loginfo("auto_trim_sails.py: recieved wind angle: %i" %wind_angle)
	wind_angle = abs((wind_angle + 180) % 360 - 180)
	rospy.loginfo("auto_trim_sails.py: simplified (0-180) wind angle: %i" %wind_angle)
	points_of_sail = [0, 45, 60, 90, 135, 180]
	sail_points = [0, 0.5, 2, 4, 8, 9]
	sail_setting = pl.interp(wind_angle, points_of_sail, sail_points)
	sail_angle = myround(sail_setting*(180/10)) # PUT CODE HERE THAT TRANSFORMS A 0-10 SCALE TO ACTUAL FULL-IN FULL-OUT ON THE SERVO
	rospy.loginfo("auto_trim_sails.py: sail servo is being set to: %i" %sail_angle)
	sail_pub.publish(UInt16(sail_angle))

def myround(x, base=5):
	return int(base * round(float(x)/base))

current_node = rospy.init_node("auto_trim_sails_node", anonymous=True)

# This ties change_desired_heading() to the desired_heading() topic (this is published to by mission_publish.py)
rospy.Subscriber("pwm_duration", UInt16, auto_trim_sails)

sail_pub = rospy.Publisher('sail', UInt16)

rospy.spin()
