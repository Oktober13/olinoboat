#!/usr/bin/env python

# lat_lon_to_UTM() takes latititude and longitude, then returns UTM values
# latlon_tools is called by sensors.py and read_mission.py
#
# You can find details on UTM here: http://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
# We think UTM is more useful than lat/lon because it is in units of meters, and the units don't change as you go north and south (unlike longitude)

# Imports necessary libraries
from pylab import *

# lat_lon_to_UTM expects to get a list of [latitude, longitude]
# lat_lon_to_UTM returns ([x, y], UTM_zone, southhemi)
#       sailbot code only ever uses [x, y], but you could use UTM_zone and southhemi to do long distance navigation
#       See wikipedia on UTM for what UTM_zone and southhemi are for
def lat_lon_to_UTM(ll):

    lat = ll[0]
    lon = ll[1]

    sm_a = 6378137.0
    sm_b = 6356752.314
    sm_EccSquared = 6.69437999013e-03

    UTMScaleFactor = 0.9996

    #Calculate the UTM zone
    zone = floor(((lon + 180.0) / 6) + 1)

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