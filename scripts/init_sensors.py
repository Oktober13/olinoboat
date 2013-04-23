#!/usr/bin/env python

# init_sensors.py initializes the sensor package, which allows rosbag to capture sensor data coming from the Arduino

# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Float64

# Imports necessary sailbot code
from hardware import sensors

def publish_data_for_bagging():
    compass_pub.publish(sensors.compass.heading)
    wind_angle_pub.publish(sensors.wind_angle.angle)
    lat_pub.publish(sensors.gps.current_y)
    lon_pub.publish(sensors.gps.current_x)

current_node = rospy.init_node("launch_sensors_for_bagging",anonymous=True)
sensors.init(current_node)

compass_pub = rospy.Publisher("compass_heading", UInt16, latch = True)
wind_angle_pub = rospy.Publisher("wind_angle", UInt16, latch = True)
lat_pub = rospy.Publisher("y_position_UTM", Float64, latch = True)
lon_pub = rospy.Publisher("x_position_UTM", Float64, latch = True)

sensors.gps.set_callback(publish_data_for_bagging)

rospy.spin()
