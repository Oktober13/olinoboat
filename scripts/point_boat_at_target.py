#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
from programming_tools import pid_controls
from hardware import sensors
from hardware import servos
import rospy

def point_boat_at_target_callback(data):
	desired_heading = data.data
	rospy.loginfo("point_boat_at_target.py: think.py sent %f" % (desired_heading))

	boat_heading = pid_controls.PID(2.0, 0.0, 1.0)

	rospy.loginfo("point_boat_at_target.py: rudder angle should be:" + str(rudder_angle))
	servos.rudder.set_position(rudder_angle)

current_node = rospy.init_node("point_boat_at_target_node",anonymous=True)
sensors.init(current_node)
servos.init(current_node)
rospy.Subscriber("desired_heading", Float64, point_boat_at_target_callback)

rospy.spin()