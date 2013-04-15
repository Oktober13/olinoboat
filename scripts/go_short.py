#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from hardware import sensors
from hardware import mission_catcher
from std_msgs.msg import String
from scipy.stats import norm

suggested_heading = [0.0]*360

def go_short_cb():
    global suggested_heading

    boat_x = sensors.gps.current_x
    boat_y = sensors.gps.current_y

    goal_point = mission_catcher.mission_goal.current_goal
    goal_x = goal_point[0]
    goal_y = goal_point[1]

    go_short_pub = rospy.Publisher('go_short_topic', String)
    rospy.loginfo("go_short.py: GPS sent (x, y) = (%f, %f)" %boat_x %boat_y)

    # (atan2(delta_x, delta_y) % (pi *2)) / pi
    # atan2 normally takes data as (y, x) - by flipping it to (x, y), we enforce the 'North = 0', 'increasing clockwise' angle structure we use elsewhere
    go_short_heading =  (atan2(goal_x-boat_x, goal_y-boat_y) % (pi*2)) * (180/pi)  # In degrees, with 0 at the nose, going clockwise

    go_short_heading_weights = [1 for x in range(360)]

    # Create a range for weights between -180 and 180 with integer steps.
    weights_range = range(-180, 180)
    std_dev = 50
    #y_vals = norm.pdf(x_vals, mean, std. deviation)
    normal_dist = norm.pdf(weights_range,0,std_dev)
    normal_dist = normal_dist / max(normal_dist)    # Normalizes the peak to 1
    
    offset = go_short_heading - ((len(normal_dist)/2) + 1)  # This is the index of normal_dist where the peak occurs
    for i in range(len(normal_dist)):
        go_short_heading_weights[(i + offset) % len(go_short_heading_weights)] = normal_dist[i]

    # go_short_pub.publish("go_short:"+str(go_short_heading_weights))
    suggested_heading = go_short_heading_weights
    rospy.loginfo("go_short.py: go_short_node suggested heading is:" + str(go_short_heading_weights))

# current_node = rospy.init_node("go_short_node",anonymous=True)
# sensors.init(current_node)

def init(think_node):
    sensors.init(think_node)
    sensors.gps.set_callback(go_short_cb)

    mission_catcher.init(think_node)