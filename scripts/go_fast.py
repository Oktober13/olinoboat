#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
from hardware import sensors
from hardware import servos
import rospy

def update_heading_on_wind_direction_change(angle):
    rospy.loginfo("Wind sensor sent %i" % (angle))

    compass_angle = sensors.compass.compass_angle
    rospy.loginfo("compass says:"+compass_angle)
    rel_angle = angle.data - compass_angle
    rel_angle = abs((rel_angle + 180) % 360 - 180)

    points_of_sail = [0, 45, 60, 90, 135, 180]    
    sail_points = [0, 0, 15, 40, 60, 80]
    sail_angle = pl.interp(rel_angle, points_of_sail, sail_angle)
    rospy.loginfo("heading angle should be:" + sail_angle)


current_node = rospy.init_node("go_fast",anonymous=True)
sensors.init(current_node)

sensors.wind_angle.set_callback(update_heading_on_wind_direction_change)

rospy.spin()