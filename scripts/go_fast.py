#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
import pylab as pl
from hardware import sensors
from std_msgs.msg import String

def pull_heading_up(array, heading):     # This array expects the form [[p1, p2], [p1, p2]...]
    print array
    for i in range(len(array)):
        for j in range(int(round(array[i][0])), int(round(array[i][1]))):
            heading[j] = 0
    return heading

def go_fast_cb():
    
    angle = sensors.wind_angle.angle

    go_fast_pub = rospy.Publisher('go_fast_topic', String)
    rospy.loginfo("Wind sensor sent %i" %angle)

    upwind_cutoff = 50	    # Degrees
    downwind_cutoff = 10    # Degrees

    go_fast_heading = [1 for x in range(360)]

    if (angle-upwind_cutoff) < 0 or (angle+upwind_cutoff) > 360:
        print 'Case 1'
        go_fast_heading = pull_heading_up([[0, (angle+upwind_cutoff) % 360],[(angle-upwind_cutoff)%360, 360],\
            [((angle+180)%360)-downwind_cutoff, ((angle+180)%360)+downwind_cutoff]], go_fast_heading)
    elif (((angle-180)%360)-downwind_cutoff) < 0 or (((angle+180)%360)+downwind_cutoff) > 360:
        print 'Case 2'
        go_fast_heading = pull_heading_up([[0, (angle+180+downwind_cutoff) % 360],[(angle-180-downwind_cutoff)%360, 360],\
            [angle-upwind_cutoff, angle+upwind_cutoff]], go_fast_heading)
    else:
        print 'Case 3'
        go_fast_heading = pull_heading_up([[((angle+180)%360)-downwind_cutoff, ((angle+180)%360)+downwind_cutoff],\
            [angle-upwind_cutoff, angle+upwind_cutoff]], go_fast_heading)

    go_fast_pub.publish(str(go_fast_heading))
    rospy.loginfo("go_fast_node suggested heading is:" + str(go_fast_heading))


current_node = rospy.init_node("go_fast_node",anonymous=True)
sensors.init(current_node)

sensors.wind_angle.set_callback(go_fast_cb)

rospy.spin()