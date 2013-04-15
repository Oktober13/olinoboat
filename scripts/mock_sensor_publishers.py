#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Float64

# def mock_sensor_publishers():

current_node = rospy.init_node("mock_sensors",anonymous=True)
# leak_detector = rospy.Publisher("leak",UInt16)
compass = rospy.Publisher("heading",UInt16)
GPS_lat = rospy.Publisher("gps_lat", Float64)	# Float32 doesn't contain enough information to hold the necessary decimal points
GPS_lon = rospy.Publisher("gps_lon", Float64)   # Float32 doesn't contain enough information to hold the necessary decimal points
wind_sensor_offset = rospy.Publisher("offset", UInt16)
wind_sensor_pwm = rospy.Publisher("pwm_duration", UInt16)

wind_sensor_offset.publish(UInt16(30)) # the 0 degree point

while not rospy.is_shutdown():
	# leak_detector.publish(UInt16(0)) # no leak
	compass.publish(UInt16(140)) # 90 degree heading
	GPS_lat.publish(Float64(42.00000011)) # Lake Waban
	GPS_lon.publish(Float64(-71.00000011)) # Lake Waban
	wind_sensor_pwm.publish(UInt16(1000)) # a random angle in the range 0-1028
	
	rospy.sleep(3)

# if __name__ == '__main__':
#     try:
#         mock_sensor_publishers()
#     except rospy.ROSInterruptException:
#         pass
