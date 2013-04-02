#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16
import random
import time
import select
import sys
import tty
from curses import ascii

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def teleop_servo_driver():
    pub1 = rospy.Publisher('servo1', UInt16)
    pub2 = rospy.Publisher('servo2', UInt16)
    rospy.init_node('teleop_controller')

    print "drive servos using wsad"
    sail_servo = servo()
    rudder_servo = servo()
    tty.setcbreak(sys.stdin.fileno())
    while not rospy.is_shutdown():
	ch = ''
        if isData():
	    ch = sys.stdin.read(1)
	    print ch
	    if ch == ascii.ESC:
		break
	if ch == 'w':
		sail_servo.change(1)
	if ch == 's':
		sail_servo.change(-1)
	if ch == 'a':
		rudder_servo.change(-1)
	if ch == 'd':
		rudder_servo.change(1)
        pub1.publish(sail_servo.angle)
        pub2.publish(rudder_servo.angle)
      	time.sleep(0.01)


class servo():
    def __init__(self):
        self.min = 0
        self.max = 180
        self.angle = (self.min+self.max)/2

    def __repr__(self):
        return 'current angle is ' + str(self.angle)

    def change(self,delta):
        self.angle += delta
	self.angle = self.angle%(self.max-self.min)

if __name__ == '__main__':
    try:
        teleop_servo_driver()
    except rospy.ROSInterruptException:
        pass
