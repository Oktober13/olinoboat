#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16
import pprint as pp

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
        rospy.loginfo("WindAngle initialized")

    def __set_offset(self, data):
        rospy.loginfo("encoder send offset signal: %i" % (data.data))
        self.offset = data.data

    def __pwm_to_wind_angle(self, data):
        rospy.loginfo("encoder sent pwm signal: %i" % (data.data))
        pp.pprint(data)
        phigh = data.data
        self.angle = (360*(phigh - self.__offset)/1024.)%360
        self.callback()     

    def set_callback(self,callback):
        self.callback = callback

    def __default_callback(self):
        rospy.loginfo("anlge updated to: %f, but no additional callback has been registered" % (self.angle))


class GPS():

    def __init__(self, node = 0):
        self.current_location = [0,0,0]
        self.callback = self.__default_callback
        self.lon_updated = 0
        self.lat_updated = 0 

        #initialize node if not loading from think code
        if node == 0:
            rospy.init_node('data_listener', anonymous=False)

        rospy.Subscriber("gps_lat", UInt16, self.__set_current_lat)
        rospy.Subscriber("gps_lon", UInt16, self.__set_current_lon)
        rospy.loginfo("GPS initialized")

    def __set_current_lat(self, data):
        rospy.loginfo("GPS sent %i latitude" % (data.data))
        self.current_lat = data.data
        self.lat_updated = 1
        self.__check_that_both_lat_and_lon_have_updated()

    def __set_current_lon(self, data):
        rospy.loginfo("GPS sent %i longitude" % (data.data))
        self.current_lon = data.data
        self.lon_updated = 1
        self.__check_that_both_lat_and_lon_have_updated()

    def __check_that_both_lat_and_lon_have_updated(self):
        if self.lat_updated and self.lon_updated:
            self.lat_updated = 0
            self.lon_updated = 0
            self.callback()

    def set_callback(self,callback):
        self.callback = callback

    def __default_callback(self):
        rospy.loginfo("GPS position updated: lat=%f and lon=%f, but no additional callback has been registered" % (self.current_lat,self.current_lon))


class Compass():

    def __init__(self, node = 0): 
        self.callback = self.__default_callback          
        self.heading = 0
    
        #initialize node if not loading from think code     
        if node == 0:
            rospy.init_node('data_listener', anonymous=False)
    

        rospy.Subscriber("heading", UInt16, self.__set_angle)
        rospy.loginfo("Compass initialized")

    def __set_angle(self, data):
        rospy.loginfo("Compass sent %i" % (data.data))
        self.heading = data.data
        self.callback()

    def set_callback(self,callback):
        self.callback = callback

    def __default_callback(self):
        rospy.loginfo("compass heading updated: %f, but no additional callback has been registered" % (self.heading))

class LeakDetector():
    def __init__(self, node=0, leak_callback=0,):
        self.leak_detected = 0 
        self.leak_callback = leak_callback or self.__default_leak_callback  

        if node == 0:
            rospy.init_node('data_listener', anonymous=False)

        rospy.Subscriber("leak",UInt16,self.__check_for_leak)
        rospy.loginfo("Leak detector initialized")

    def __check_for_leak(self,data):
        if data.data > 1:
            self.leak_detected = 1
            self.leak_callback()
        else:
            rospy.loginfo("clear, no leak")
            self.leak = 0

    def __default_leak_callback(self):
        rospy.loginfo("Leak detected, but no callback registered")

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

 
if __name__ == '__main__':
   pass