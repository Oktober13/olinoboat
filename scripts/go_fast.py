#!/usr/bin/env python

# go_fast.py provides functions for think.py
# At a high level, go_fast.py tells think.py how much it would like to go in every direction around the boat
#
# 04/14/13 - The logic behind the go_fast.py takes the wind direction, then tells the boat that go_fast.py really doesn't want to go straight upwind or straight downwind
#       The boat is told it can't go within 50 degrees (upwind_cutoff, may be adjusted) of upwind
#       The boat is told it can't go within 10 degrees (downwind_cutoff, may be adjusted) of downwind
#       go_fast.py does this by setting the directions the boat shouldn't go as 0, and directions it can go as 1 (see comments directly below for details)
#
# When go_fast.py is initialized by think.py, it updates the suggested heading based on wind every time the computer gets new wind data from the wind sensor
#       go_fast and go_short both output a 360 element list (called suggested_heading)
#       These lists correspond to degrees - the first element is 0 degrees (straight forward), the 90th element is 90 deg (starboard) etc.
#       The number stored in the list element is how strong the boat wants to go to that angle (from 0-1)
#       For example, if the boat wants to go to 180 degrees (backwards) the 180th element in the list should be 1 (ish)
#       If the boat really doesn't want to go to 270 degrees (port) the 270th element in the list should be 0 (ish)

# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
import pylab as pl
from std_msgs.msg import String

# Imports necessary sailbot code
from hardware import sensors

# Initializes suggested_heading as a 360 element list of integers
suggested_heading = [1]*360

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
    #rospy.loginfo("go_fast.py:go_fast_node suggested heading is:" + str(go_fast_heading_weights))
    go_fast_pub.publish(str(go_fast_heading_weights))


# inhibit_bad_headings takes in a 360 element list (wind_heading) and a 2 element array (inhibited heading, for example [25, 30])
# In this example, it sets the 25th, 26th, 27th, 28th, and 29th elements of windheading (wind_heading[25:30]) to 0
def inhibit_bad_headings(inhibited_array, wind_heading):
    print inhibited_array
    for i in range(len(inhibited_array)):
        for j in range(int(round(inhibited_array[i][0])), int(round(inhibited_array[i][1]))):
            wind_heading[j] = 0
    return wind_heading


def init(think_node):
    sensors.init(think_node)
    sensors.wind_angle.set_callback(go_fast_cb)

