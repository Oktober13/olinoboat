#!/usr/bin/env python

# go_short.py provides functions for think.py
# At a high level, go_short.py tells think.py how much it would like to go in every direction around the boat
#
# 04/14/13 - The logic behind the go_short.py takes the boat position (GPS) and the next waypoint position (GPS) and tells think.py it really wants to go straight at the waypoint
#       go_short.py does this by making a normal distribution (https://en.wikipedia.org/wiki/File:Standard_deviation_diagram.svg) around the desired heading
#       See documentation for more details on the normal distribution suggestion
#
# When go_short.py is initialized by think.py, it updates the suggested headings based on boat position every time the computer gets new GPS data
#       go_fast and go_short both output a 360 element list (called suggested_heading)
#       These lists correspond to degrees - the first element is 0 degrees (straight forward), the 90th element is 90 deg (starboard) etc.
#       The number stored in the list element is how strong the boat wants to go to that angle (from 0-1)
#       For example, if the boat wants to go to 180 degrees (backwards) the 180th element in the list should be 1 (ish)
#       If the boat really doesn't want to go to 270 degrees (port) the 270th element in the list should be 0 (ish)

# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import String
from scipy.stats import norm
from math import atan2, pi

# Imports necessary sailbot code
from hardware import sensors
from hardware import mission_catcher

# Initializes suggested_heading as a 360 element list of floats
suggested_heading = [0.0]*360

def go_short_cb():
    global suggested_heading

    boat_x = sensors.gps.current_x
    boat_y = sensors.gps.current_y

    goal_waypoint = mission_catcher.mission_goal.current_goal
    goal_x = goal_waypoint[0]
    goal_y = goal_waypoint[1]
    rospy.loginfo( "go_short.py: Goal waypoint is " + str(goal_x) + ', ' + str(goal_y))

    go_short_pub = rospy.Publisher('go_short_topic', String)
    rospy.loginfo("go_short.py: GPS sent (x, y) = (%f, %f)" %(boat_x, boat_y))

    # Finds the angle from our boat to the next waypoint
        # atan2 format : (atan2(delta_x, delta_y) % (pi *2)) / pi
        # atan2 normally takes data as (y, x) - by flipping it to (x, y), we enforce the 'North = 0', 'increasing clockwise' angle structure we use elsewhere
    go_short_optimal_heading =  (atan2(goal_x-boat_x, goal_y-boat_y) % (pi*2)) * (180/pi)  # In degrees, with 0 at the nose, going clockwise

    go_short_heading_weights = [1.0 for x in range(360)]

    # Creats a normal distribution suggestion centered around 0
        # normal distribution format : y_vals = norm.pdf(x_vals, mean, std. deviation)
    weights_range = range(-180, 180)
    std_dev = 50
    normal_dist = norm.pdf(weights_range,0,std_dev)
    normal_dist = normal_dist / max(normal_dist)    # Normalizes the peak to 1

    # Offsets the normal distribution suggestion to point at the waypoint    
    offset = int(go_short_optimal_heading - ((len(normal_dist)/2) + 1))  # This is the index of normal_dist where the peak occurs
    for i in range(len(normal_dist)):
        go_short_heading_weights[(i + offset) % len(go_short_heading_weights)] = normal_dist[i]

    suggested_heading = go_short_heading_weights


def init(think_node):
    sensors.init(think_node)
    sensors.gps.set_callback(go_short_cb)

    mission_catcher.init(think_node)
