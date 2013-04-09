#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16

current_node = rospy.init_node("mock_sensors",anonymous=True)
# leak_detector = rospy.Publisher("leak",UInt16)
compass = rospy.Publisher("heading",UInt16)
GPS_lat = rospy.Publisher("gps_lat", UInt16)
GPS_lon = rospy.Publisher("gps_lon", UInt16)
wind_sensor_offset = rospy.Publisher("offset", UInt16)
wind_sensor_pwm = rospy.Publisher("pwm_duration", UInt16)

wind_sensor_offset.publish(UInt16(30)) # the 0 degree point

while not rospy.is_shutdown():
	# leak_detector.publish(UInt16(0)) # no leak
	compass.publish(UInt16(70)) # 70 degree heading
	GPS_lat.publish(UInt16(12.34567890)) # who knows where this is?
	GPS_lon.publish(UInt16(98.76543211)) 
	wind_sensor_pwm.publish(UInt16(800)) # a random angle in the range 0-1028
	
	rospy.sleep(3)
