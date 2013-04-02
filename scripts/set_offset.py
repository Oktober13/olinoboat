import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
from std_msgs.msg import UInt16


def pwm_callback(data):
    offset = data.data
    print offset
    offset_publisher = rospy.Publisher("pwm_offset", UInt16, latch = True)
    offset_publisher.publish(offset)

if __name__ == "__main__":
    rospy.init_node('encoder_offset_setter')
    rospy.Subscriber("pwm_duration", UInt16, pwm_callback)
    rospy.sleep(2)

    

