#!/usr/bin/env python

# sensors.py contains a class that needs to be initialized by another piece of code to use
# More information about python 'classes' here: http://www.tutorialspoint.com/python/python_classes_objects.htm
#
# sensors.py is initialized by every piece of code that needs the sensor data
#       04/14/2013 - Files that use sensors.py
#               mission.py -- go_fast.py -- go_short.py -- maintain_fast_sail_angle.py -- point_boat_at_target.py
#
# The point of using classes like this is that every time the sensor data is published, the data is caught in this class (once intiailized) and then can be used
#       See examples in every piece of code that imports sensors.py

# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
from pylab import *
from std_msgs.msg import UInt16
from std_msgs.msg import Float64

# Imports necessary sailbot code
from programming_tools import latlon_tools

compass=None
gps=None
wind_angle=None
leak_detector=None

class WindAngle():

    def __init__(self, offset = 0, node = 0):
        self.__offset = offset
        self.angle = 0
        self.callback = self.__default_callback

        #initialize node if not running from the think code
        if node == 0:
            rospy.init_node('data_listener', anonymous=False)

        #set up subscribers to ("topic", DataType, callback_function)
        rospy.Subscriber("offset", UInt16, self.__set_offset)
        rospy.Subscriber("pwm_duration", UInt16, self.__pwm_to_wind_angle)
        rospy.loginfo("sensors.py: WindAngle initialized")

    def __set_offset(self, data):
        rospy.loginfo("sensors.py: wind encoder send offset signal: %i" % (data.data))
        self.offset = data.data

    def __pwm_to_wind_angle(self, data):
        rospy.loginfo("sensors.py: wind encoder sent pwm signal: %i" % (data.data))
        phigh = data.data
        self.angle = (360*(phigh - self.__offset)/1024)%360
        self.callback()

    def set_callback(self,callback):
        self.callback = callback

    def __default_callback(self):
        rospy.loginfo("sensors.py: Wind angle updated to: %f, but no additional callback has been registered" % (self.angle))


class GPS():    # If you want to do any really long term missions with this code, the UTM zones (currently unused) must be integrated. Wikipedia UTM for more information

    def __init__(self, node = 0):
        self.current_location = [0,0,0]
        self.callback = self.__default_callback
        self.current_x = 0
        self.current_y = 0
        self.x_updated = 0
        self.y_updated = 0 

        #initialize node if not loading from think code
        if node == 0:
            rospy.init_node('data_listener', anonymous=False)

        rospy.Subscriber("gps_lon", Float64, self.__set_current_x)
        rospy.Subscriber("gps_lat", Float64, self.__set_current_y)
        rospy.loginfo("sensors.py: GPS initialized")

    def __set_current_x(self, data):
        rospy.loginfo("sensors.py: GPS sent %i longitude" % (data.data))
        self.current_x = latlon_tools.lat_lon_to_UTM([0, data.data])[0][1]
        self.x_updated = 1
        self.__check_that_both_x_and_y_have_updated()

    def __set_current_y(self, data):
        rospy.loginfo("sensors.py: GPS sent %i latitude" % (data.data))
        self.current_y = latlon_tools.lat_lon_to_UTM([data.data, 0])[0][0]
        self.y_updated = 1
        self.__check_that_both_x_and_y_have_updated()

    def __check_that_both_x_and_y_have_updated(self):
        if self.x_updated and self.y_updated:
            self.x_updated = 0
            self.y_updated = 0
            self.callback()

    def set_callback(self,callback):
        self.callback = callback

    def __default_callback(self):
        rospy.loginfo("sensors.py: GPS position updated: x=%f and y=%f, but no additional callback has been registered" % (self.current_x,self.current_y))


class Compass():

    def __init__(self, node = 0): 
        self.callback = self.__default_callback          
        self.heading = 0
    
        #initialize node if not loading from think code     
        if node == 0:
            rospy.init_node('data_listener', anonymous=False)

        rospy.Subscriber("heading", UInt16, self.__set_angle)
        rospy.loginfo("sensors.py: Compass initialized")

    def __set_angle(self, data):
        rospy.loginfo("sensors.py: Compass sent %i" % (data.data))
        self.heading = data.data
        self.callback()

    def set_callback(self,callback):
        self.callback = callback

    def __default_callback(self):
        rospy.loginfo("sensors.py: compass heading updated: %f, but no additional callback has been registered" % (self.heading))

class LeakDetector():
    def __init__(self, node=0, leak_callback=0,):
        self.leak_detected = 0 
        self.leak_callback = leak_callback or self.__default_leak_callback  

        if node == 0:
            rospy.init_node('data_listener', anonymous=False)

        rospy.Subscriber("leak",UInt16,self.__check_for_leak)
        rospy.loginfo("sensors.py: Leak detector initialized")

    def __check_for_leak(self,data):
        if data.data > 1:
            self.leak_detected = 1
            self.leak_callback()
        else:
            rospy.loginfo("sensors.py: clear, no leak")
            self.leak = 0

    def __default_leak_callback(self):
        rospy.loginfo("sensors.py: Leak detected, but no callback registered")

    def set_callback(self,callback):
        self.leak_callback = callback


def init(node):
    global wind_angle
    global gps
    global compass
    # global leak_detector

    wind_angle = WindAngle(0,node)
    gps = GPS(node)
    compass = Compass(node)
    # leak_detector = LeakDetector(node)