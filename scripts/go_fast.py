#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
from hardware import sensors
import rospy
import pprint
pp = pprint.PrettyPrinter(indent=4)

def on_wind_direction_change(angle):
    rospy.loginfo("Compass sent %i" % (angle))

current_node = rospy.init_node("go_fast",anonymous=True)
sensors.init(current_node)
sensors.wind_angle.set_callback(on_wind_direction_change)

rospy.spin()