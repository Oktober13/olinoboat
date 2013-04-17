#!/usr/bin/env python
import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
from std_msgs.msg import Float64

def callback1(data):
    rospy.loginfo(rospy.get_name() + ": callback1 heard %f" % data.data)
    print data.data + passer
    
def callback2(data):
    global passer
    rospy.loginfo(rospy.get_name() + ": callback2 heard %f" % data.data)
    passer = data.data

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("gps_lat", Float64, callback1)
    rospy.Subscriber("gps_lon", Float64, callback2)
    rospy.spin()


if __name__ == '__main__':
    print 'Hey!'
    passer = 0
    listener()