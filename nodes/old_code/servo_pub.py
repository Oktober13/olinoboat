#!/usr/bin/env python
from low_level_ros import publisher, subscriber
import rospy
rospy.init_node("servo_stuff")

def listen(data):
	print data

publisher.initialize("servo1")
publisher.initialize("servo2")
subscriber.initialize("potentiometer",listen)

while(True):
	publisher.publish("hey there")
	rospy.sleep(1)


