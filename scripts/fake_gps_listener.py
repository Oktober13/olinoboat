#!/usr/bin/env python
import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
from std_msgs.msg import Float32
import rosgraph.masterapi


def callback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %f" % data.data)
    print data.data

    master = rosgraph.masterapi.Master('/rostopic')
    print master.getPublishedTopics('/') 

    # rostopic echo "lon_goal"

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("lat_goal", Float32, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()