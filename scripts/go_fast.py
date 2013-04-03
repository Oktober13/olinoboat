#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
from hardware import sensors
from hardware import servos
import rospy
from pylab as pl

def sail_update_on_wind_direction_change(angle):
    rospy.loginfo("Wind sensor sent %i" % (angle))
    
    compass_angle = sensors.compass.compass_angle
    rel_angle = angle.data - compass_angle
	rel_angle = abs((rel_angle + 180) % 360 - 180)

	points_of_sail = [0, 45, 60, 90, 135, 180]    
	sail_points = [0, 0, 15, 40, 60, 80]
	sail_angle = pl.interp(rel_angle, points_of_sail, sail_angle)

	servos.sail.set_position(sail_angle)

current_node = rospy.init_node("go_fast",anonymous=True)
sensors.init(current_node)

sensors.wind_angle.set_callback(sail_update_on_wind_direction_change)

rospy.spin()