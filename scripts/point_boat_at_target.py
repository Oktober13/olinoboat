#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
from programming_tools import pid_controls
from hardware import sensors
from hardware import servos
import rospy

def point_boat_at_target_on_compass_chage(data):
	compass_heading = data.data
	rospy.loginfo("point_boat_at_target.py: Boat is pointing at %f degrees" % (compass_heading))



	rospy.loginfo("point_boat_at_target.py: rudder angle should be:" + str(rudder_angle))
	servos.rudder.set_position(rudder_angle)

def change_desired_heading(data):	# This function is subscribed to the think code - as soon as our arbiter decides a new direction we 
	global boat_heading
	compass_heading = sensors.compass.heading
	desired_compass_heading = (compass_heading + data.data)%360
	boat_heading.setPoint(desired_compass_heading)	# The PID setpoint is updated
	rospy.loginfo("point_boat_at_target.py: Orint boat to compass heading %i" %desired_compass_heading)	

current_node = rospy.init_node("point_boat_at_target_node",anonymous=True)
sensors.init(current_node)
servos.init(current_node)

rospy.Subscriber("desired_heading", UInt16, change_desired_heading)

sensors.compass.set_callback(point_boat_at_target_on_compass_chage)
boat_heading = pid_controls.PID(2.0, 0.0, 1.0)

rospy.spin()