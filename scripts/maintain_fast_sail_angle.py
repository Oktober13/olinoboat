#!/usr/bin/env python

# maintain_fast_sail_angle.py is activated every time the wind data changes, and then it updates the sail position
#
# 04/14/2013 - Current logic takes the wind angle relative to the boat, then sets the sail using the lookup table
#		See documentation for how the lookup table works
#		For the future - this lookup table could be tuned to your boat, or a new method of updating the sails could be used

# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
import pylab as pl

# Imports necessary sailbot code
from hardware import sensors
from hardware import servos

def sail_update_on_wind_direction_change():
	wind_angle = sensors.wind_angle.angle
	rospy.loginfo("fast_sail_angle.py: Wind sensor sent %f" % (wind_angle))

	# This command takes an angle and converts it to 0-180 range
	#		necessary because it doesn't matter which side the wind is coming from for sail setting - wind 90 deg. off the starboard OR port bow both result in the same sail setting
	wind_angle = abs((wind_angle + 180) % 360 - 180)

	points_of_sail = [0, 45, 60, 90, 135, 180]    
	sail_points = [0, 0, 15, 40, 60, 80]
	sail_angle = pl.interp(wind_angle, points_of_sail,sail_points)
	rospy.loginfo("fast_sail_angle.py: sail angle should be:" + str(sail_angle))

	# Sets the sail servo
	servos.sail.set_position(sail_angle)

current_node = rospy.init_node("maintain_fast_sail_position",anonymous=True)
sensors.init(current_node)
servos.init(current_node)

# This line calls sail_update_on_wind_direction_change() every time the wind direction changes
sensors.wind_angle.set_callback(sail_update_on_wind_direction_change)

rospy.spin()
