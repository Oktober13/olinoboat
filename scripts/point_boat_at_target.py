#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16
from programming_tools import pid_controls
from hardware import sensors
from hardware import servos

def point_boat_at_target_on_compass_change():
	compass_heading = sensors.compass.heading
	rospy.loginfo("point_boat_at_target.py: Boat is pointing at %f degrees and should be at %i degrees" %(compass_heading, boat_heading.getPoint()))

	pid_command = boat_heading.update(compass_heading)
	rospy.loginfo('point_boat_at_target.py: pid command %i' %pid_command)

	rudder_bound = 45	# This is, in degrees, the farthest that the rudder should go to either side
	rudder_angle = clamp(pid_command, -rudder_bound, rudder_bound)	# This works assuming the rudder has 'S = 0' and angle increasing CCW
	# That way when the boat turns left (pid_command is negative) then the rudder turns clockwise, making a left turn

	rospy.loginfo("point_boat_at_target.py: rudder angle is being set to: %i" %rudder_angle)
	servos.rudder.set_position(rudder_angle)

def change_desired_heading(data):	# This function is subscribed to the think code - as soon as our arbiter decides a new direction we 
	global boat_heading
	compass_heading = sensors.compass.heading
	desired_compass_heading = (compass_heading + data.data) % 360
	boat_heading.setPoint(desired_compass_heading)	# The PID setpoint is updated
	rospy.loginfo("point_boat_at_target.py: Boat should reorient to compass heading %i" %desired_compass_heading)	

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

current_node = rospy.init_node("point_boat_at_target_node",anonymous=True)
sensors.init(current_node)
servos.init(current_node)

rospy.Subscriber("desired_heading", UInt16, change_desired_heading)

sensors.compass.set_callback(point_boat_at_target_on_compass_change)
boat_heading = pid_controls.PID(1.0, 0.0, 0.0)

rospy.spin()