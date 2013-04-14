#!/usr/bin/env python
import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
from std_msgs.msg import Float64


def callback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %f" % data.data)
    print data.data

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("gps_lat", Float64, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()