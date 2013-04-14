#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from hardware import sensors
from std_msgs.msg import String
from scipy.stats import norm

suggested_heading = [1]*360

def go_short_cb():
    global suggested_heading

    boat_x = sensors.gps.current_x
    boat_y = sensors.gps.current_y

    go_short_pub = rospy.Publisher('go_short_topic', String)
    rospy.loginfo("go_short.py: GPS sent (x, y) = (%f, %f)" %boat_x %boat_y)


    go_short_heading = 15   # In degrees, with 0 at the nose, going clockwise

    go_short_heading_weights = [1 for x in range(360)]

    # Create a range for weights between -180 and 180 with integer steps.
    weights_range = range(-180, 180)
    std_dev = 50
    #y_vals = norm.pdf(x_vals, mean, std. deviation)
    normal_dist = norm.pdf(weights_range,0,std_dev))
    normal_dist = normal_dist / max(normal_dist)    # Normalizes the peak to 1
    
    offset = go_short_heading - ((len(normal_dist)/2) + 1)  # This is the index of normal_dist where the peak occurs
    for i in range(len(normal_dist)):
        go_short_heading_weights[i - offset] = normal_dist[i]

    # go_short_pub.publish("go_short:"+str(go_short_heading_weights))
    suggested_heading = go_short_heading_weights
    rospy.loginfo("go_short.py: go_short_node suggested heading is:" + str(go_short_heading_weights))

# current_node = rospy.init_node("go_short_node",anonymous=True)
# sensors.init(current_node)

def init(think_node):
    sensors.init(think_node)
    sensors.wind_angle.set_callback(go_short_cb)
