#!/usr/bin/env python

# mock_sensors_publishers.py takes the place of the alaMode
# It publishes fake data (replicating real data) to the same topics that the alaMode publishes its real data to
# You can insert your own fake data into the publishers in mock_sensor_publishers.py, then test your code
# The reason you would do this is to try and test your code quickly, when you know what you think the output should be

# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Float64

current_node = rospy.init_node("mock_sensors",anonymous=True)

# leak_detector = rospy.Publisher("leak",UInt16)
compass = rospy.Publisher("heading",UInt16)
GPS_lat = rospy.Publisher("gps_lat", Float64)	# Float32 doesn't contain enough information to hold the necessary decimal points
GPS_lon = rospy.Publisher("gps_lon", Float64)   # Float32 doesn't contain enough information to hold the necessary decimal points
wind_sensor_offset = rospy.Publisher("offset", UInt16)
wind_sensor_pwm = rospy.Publisher("pwm_duration", UInt16)

wind_sensor_offset.publish(UInt16(0))			# the 0 degree point

while not rospy.is_shutdown():
	# leak_detector.publish(UInt16(0)) 			# No leak
	compass.publish(UInt16(140))				# 90 degree heading
	GPS_lat.publish(Float64(42.00000011))		# Lake Waban-ish, choose your own lat/lon with google maps
	GPS_lon.publish(Float64(-71.00000011))
	wind_sensor_pwm.publish(UInt16(300)) 		# A random angle in the range 0-1028
	
	rospy.sleep(3)
