#!/usr/bin/env python

# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16

# Imports necessary sailbot code
from programming_tools import pid_controls
from hardware import sensors
from hardware import servos

# This function using PID control to get the boat pointed at the correct compass heading
# This function updates every time the compass heading updates
# You can find more information about pid control here: http://www.csimn.com/CSI_pages/PIDforDummies.html
# If you read the paper:
#		The Setpoint of the PID control is the command 'boat_heading.setPoint(desired_compass_heading)'
#		The Process Variable is the compass data, seen in the command 'boat_heading.update(compass_heading)'
#		The Controller Output, in this case, is called pid_command, and we use that to set our rudder angle
def point_boat_at_target_on_compass_change():
	compass_heading = sensors.compass.heading
	rospy.loginfo("point_boat_at_target.py: Boat is pointing at %f degrees and should be at %i degrees" %(compass_heading, boat_heading.getPoint()))

	pid_command = boat_heading.update(compass_heading)
	rospy.loginfo('point_boat_at_target.py: pid command %i' %pid_command)

	rudder_bound = 45	# This is, in degrees, the farthest that the rudder should go to either side. Can be reset here
	rudder_angle = clamp(pid_command, -rudder_bound, rudder_bound)	# This works assuming the rudder has 'S = 0' and angle increasing CCW
	# That way when the boat turns left (pid_command is negative) then the rudder turns clockwise, making a left turn

	rospy.loginfo("point_boat_at_target.py: rudder angle is being set to: %i" %rudder_angle)
	servos.rudder.set_position(rudder_angle)

# This function is subscribed to the think code - as soon as our arbiter decides a new direction we set that as the Setpoint of our PID control loop
def change_desired_heading(data):
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

# This ties change_desired_heading() to the desired_heading() topic (this is published to by mission_publish.py)
rospy.Subscriber("desired_heading", UInt16, change_desired_heading)

# This ties point_boat_at_target_on_compass_change() to a change in compass data
sensors.compass.set_callback(point_boat_at_target_on_compass_change)

# You can tune the PID control by altering these constants
# Tuning a PID controller is very useful, and helps the robot control the boat better
# Some info about tuning can be found here, but googling will turn up more if this isn't useful to you
#		https://controls.engin.umich.edu/wiki/index.php/PIDTuningClassical#Trial_and_Error
# Essentially, making P control larger will get faster responses, but cause overshoot
# Higher D control will help prevent overshoot
# Higher I control will help if the boat is getting close to the desired angle but never quite gets there
boat_heading = pid_controls.PID(1.0, 0.0, 0.0)

rospy.spin()