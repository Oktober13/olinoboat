#!/usr/bin/env python

# set_offset.py is used for setting the wind angle to a new zero
# This is necessary because the 'zero' point of the encoder is not necessarily straight forward on the boat
#		You run set_offset.py when the wind sensor is pointing directly forward, and the think code will treat that as the new 'zero' for the wind sensor

# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16

def pwm_callback(data):
    offset = data.data
    offset_angle = (offset * 360 / 1024) % 360
    rospy.loginfo("set_offset.py: The wind sensor's reset so that the offset is %i out of 1024, or %i degrees" %(offset, offset_angle))
    offset_publisher = rospy.Publisher("pwm_offset", UInt16, latch = True)
    offset_publisher.publish(offset)

if __name__ == "__main__":
    rospy.init_node('encoder_offset_setter')
    rospy.Subscriber("pwm_duration", UInt16, pwm_callback)
    rospy.sleep(2)

    

