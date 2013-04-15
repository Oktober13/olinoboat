#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from pylab import *
from std_msgs.msg import UInt16
from std_msgs.msg import Float64
# import pprint as pp

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
        # pp.pprint(data)
        phigh = data.data
        self.angle = (360*(phigh - self.__offset)/1024.)%360
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
        self.current_x = lat_lon_to_UTM([0, data.data])[0][1]
        self.x_updated = 1
        self.__check_that_both_x_and_y_have_updated()

    def __set_current_y(self, data):
        rospy.loginfo("sensors.py: GPS sent %i latitude" % (data.data))
        self.current_y = lat_lon_to_UTM([data.data, 0])[0][0]
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


def lat_lon_to_UTM(ll):

    lat = ll[0]
    lon = ll[1]

    sm_a = 6378137.0
    sm_b = 6356752.314
    sm_EccSquared = 6.69437999013e-03

    UTMScaleFactor = 0.9996

    #Ccalculate the UTM zone
    zone = floor(((lon + 180.0) / 6) + 1)

    #TODO: Flags invalid GPS points?

    # Convert lat and lon to radians
    lat = lat/180*pi
    lon = lon/180*pi


    # UTMCentralMeridian
    # Determines the central meridian for the given UTM zone.
    # Inputs:
    #   zone - An integer value designating the UTM zone, range [1,60].
    # Returns:
    #   The central meridian for the given UTM zone, in radians
    #   Range of the central meridian is the radian equivalent of [-177,+177].

    cmeridian = (-183.0 + (zone * 6.0)) / 180*pi


    # ArcLengthOfMeridian
    # Computes the ellipsoidal distance from the equator to a point at a
    # given latitude, in meters
    # Inputs:
    #   lat - Latitude of the point, in radians.
    # Globals:
    #   sm_a - Ellipsoid model major axis.
    #   sm_b - Ellipsoid model minor axis.

    n = (sm_a - sm_b) / (sm_a + sm_b)
    alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n, 4.0) / 64.0))
    beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0) + (-3.0 * pow(n, 5.0) / 32.0)
    gamma = (15.0 * pow(n, 5.0) / 16.0) + (-15.0 * pow(n, 4.0) / 32.0)
    delta = (-35.0 * pow(n, 3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0)
    epsilon = (315.0 * pow(n, 4.0) / 512.0)

    arcLength = alpha * (lat + (beta * sin (2.0 * lat)) + (gamma * sin (4.0 * lat)) + (delta * sin (6.0 * lat)) + (epsilon * sin (8.0 * lat)))


    # MapLatLonToXY
    # Converts a latitude/longitude pair to x and y coordinates in the
    # Transverse Mercator projection. Transverse Mercator is not UTM until scaled
    # Inputs:
    #   lat - Latitude of the point, in radians.
    #   lon - Longitude of the point, in radians.
    #   cmeridian - Longitude of the central meridian to be used, in radians.

    ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0)
    nu2 = ep2 * pow (cos(lat), 2.0)
    N = pow(sm_a, 2.0) / (sm_b * sqrt (1 + nu2))

    t = tan(lat)
    t2 = t * t
    tmp = (t2 * t2 * t2) - pow(t, 6.0)

    l = lon - cmeridian


    # Precalculate coefficients for l**n in the equations below
    # so a normal human being can read the expressions for easting
    # and northing

    l3coef = 1.0 - t2 + nu2
    l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2)
    l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2
    l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2
    l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2)
    l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2)


    # Calculate easting (x)

    x = N * cos(lat) * l\
    + (N / 6.0 * pow(cos(lat), 3.0) * l3coef * pow(l, 3.0))\
    + (N / 120.0 * pow(cos(lat), 5.0) * l5coef * pow(l, 5.0))\
    + (N / 5040.0 * pow(cos(lat), 7.0) * l7coef * pow(l, 7.0))


    # Calculate northing (y)

    y = arcLength\
    + (t / 2.0 * N * pow(cos(lat), 2.0) * pow(l, 2.0))\
    + (t / 24.0 * N * pow(cos(lat), 4.0) * l4coef * pow(l, 4.0))\
    + (t / 720.0 * N * pow(cos(lat), 6.0) * l6coef * pow(l, 6.0))\
    + (t / 40320.0 * N * pow(cos(lat), 8.0) * l8coef * pow(l, 8.0))


    # Adjust easting and northing for UTM system.

    x = round(x * UTMScaleFactor + 500000.0)
    y = round(y * UTMScaleFactor)
    if (y < 0.0):
        y = y + 10000000.0
    if (lat < 0):
        southhemi = 1
    else:
        southhemi = 0

    return ([x, y], zone, southhemi)
 

if __name__ == '__main__':
   pass