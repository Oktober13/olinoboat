#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import Float32


def fake_gps_goal():

    lat = 42.00000001
    lon = -71.00000001

    lat_pub = rospy.Publisher('lat_goal', Float32)
    lon_pub = rospy.Publisher('lon_goal', Float32)
    rospy.init_node('fake_gps_goal')
    while not rospy.is_shutdown():
        lat_pub.publish(lat)
        lon_pub.publish(lon)
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        fake_gps_goal()
    except rospy.ROSInterruptException:
        pass
