#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
import pylab as pl
from hardware import sensors
from std_msgs.msg import String

suggested_heading = [1]*360

def inhibit_bad_headings(inhibited_array, wind_heading):     # This array expects the form [[p1, p2], [p1, p2]...]
    print inhibited_array
    for i in range(len(inhibited_array)):
        for j in range(int(round(inhibited_array[i][0])), int(round(inhibited_array[i][1]))):
            wind_heading[j] = 0
    return wind_heading

def go_fast_cb():
    global suggested_heading

    angle = sensors.wind_angle.angle

    go_fast_pub = rospy.Publisher('go_fast_topic', String)
    rospy.loginfo("go_fast.py:Wind sensor sent %i" %angle)

    upwind_cutoff = 50	    # Degrees
    downwind_cutoff = 10    # Degrees

    go_fast_heading_weights = [1 for x in range(360)]

    if (angle-upwind_cutoff) < 0 or (angle+upwind_cutoff) > 360:
        print 'Case 1'
        go_fast_heading_weights = inhibit_bad_headings([[0, (angle+upwind_cutoff) % 360],[(angle-upwind_cutoff)%360, 360],\
            [((angle+180)%360)-downwind_cutoff, ((angle+180)%360)+downwind_cutoff]], go_fast_heading_weights)
    elif (((angle-180)%360)-downwind_cutoff) < 0 or (((angle+180)%360)+downwind_cutoff) > 360:
        print 'Case 2'
        go_fast_heading_weights = inhibit_bad_headings([[0, (angle+180+downwind_cutoff) % 360],[(angle-180-downwind_cutoff)%360, 360],\
            [angle-upwind_cutoff, angle+upwind_cutoff]], go_fast_heading_weights)
    else:
        print 'Case 3'
        go_fast_heading_weights = inhibit_bad_headings([[((angle+180)%360)-downwind_cutoff, ((angle+180)%360)+downwind_cutoff],\
            [angle-upwind_cutoff, angle+upwind_cutoff]], go_fast_heading_weights)

    # go_fast_pub.publish("go_fast:"+str(go_fast_heading_weights))
    suggested_heading = go_fast_heading_weights
    rospy.loginfo("go_fast.py:go_fast_node suggested heading is:" + str(go_fast_heading_weights))


# current_node = rospy.init_node("go_fast_node",anonymous=True)
# sensors.init(current_node)


def init(think_node):
    sensors.init(think_node)
    sensors.wind_angle.set_callback(go_fast_cb)