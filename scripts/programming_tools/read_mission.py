#!/usr/bin/env python

# read_mission.py provides functions that are called by mission.py
#
# The functions in read_mission.py look for a mission_list.csv file in the olinoboat folder, then interpret the missions found there
# More details on the mission_list.csv file can be found in mission_list_csv_howto.txt (in the olinoboat folder)
#
# Future note - the interpret_mission_file() function can be edited to give new functionality
#		As of 04/14/2013, interpret_mission_file() can take a GPS point and set it as a waypoint (mission type 1)
#		As of 04/14/2013, interpret_mission_file() can take a buoy GPS point, then make 3 waypoints around the buoy so that the boat will round it (mission type 2)
# 		The power of interpret_mission_file() is that you can use simple user inputs (a single GPS point) to make a complex mission. Buoy rounding is a simple example of that

# Imports necessary libraries
import csv
import os.path
from ast import literal_eval
from math import atan2, pi, sin, cos

# Imports necessary sailbot code
from programming_tools import latlon_tools


def interpret_mission_file(mission_list):
	boat_goal_points = []
	for i in range(len(mission_list)):
		mission = mission_list[i]			# The format for a mission should be ['type of mission', 'information passing parameter', 'goal_latitude', 'goal_longitude']
		if mission[0] == 1:					# 'type of mission' = 1 means do waypoint following
			goal_UTM = latlon_tools.lat_lon_to_UTM([float(mission[2]), float(mission[3])])
			goal_x_y = goal_UTM[0]
			boat_goal_points.append(goal_x_y)

		elif mission[0] == 2 and i == 0:	# reject the 'round buoy' mission if it is the first mission, because there is original directino to interpret around
			print 'Rejected' + str(mission) + "as a mission because you can't round the buoy as the first mission - you need to give the boat a starting waypoint that it can get point itself at the buoy from"
		elif mission[0] == 2:
			radius_of_round_buoy = 2		# this will create waypoints with a radius of 2 meters around the buoy

			buoy_UTM = latlon_tools.lat_lon_to_UTM([float(mission[2]), float(mission[3])])
			buoy_x_y = buoy_UTM[0]

			prev_wp_UTM = latlon_tools.lat_lon_to_UTM([float(mission_list[i-1][2]), float(mission_list[i-1][3])])
			prev_wp_x_y = prev_wp_UTM[0]

			angle_to_buoy = atan2(buoy_x_y[1] - prev_wp_x_y[1], buoy_x_y[0] - prev_wp_x_y[0])
			if mission[1] == 1:				# round the buoy clockwise
				offset1 = pi/2
				offset2 = 0
				offset3 = -pi/2
			else:							# round the buoy counterclockwise
				offset1 = -pi/2
				offset2 = 0
				offset3 = pi/2

			goal1_x_y = [buoy_x_y[0] + (radius_of_round_buoy * cos(angle_to_buoy + offset1)), buoy_x_y[1] + radius_of_round_buoy * sin(angle_to_buoy + offset1)]
			goal2_x_y = [buoy_x_y[0] + (radius_of_round_buoy * cos(angle_to_buoy + offset2)), buoy_x_y[1] + radius_of_round_buoy * sin(angle_to_buoy + offset2)]
			goal3_x_y = [buoy_x_y[0] + (radius_of_round_buoy * cos(angle_to_buoy + offset3)), buoy_x_y[1] + radius_of_round_buoy * sin(angle_to_buoy + offset3)]

			boat_goal_points.append(goal1_x_y)
			boat_goal_points.append(goal2_x_y)
			boat_goal_points.append(goal3_x_y)

		# elif mission[0] == 3:		# New people can define their own mission interpretations here
			# goal_x_y = [mission[2] + your stuff, mission[3] + your stuff]
			# boat_goal_points.append(goal_x_y)

		else:
			print str(mission) + " has an unknown type of mission"

	return boat_goal_points


def read_mission_csv():
	filename = "mission_file.csv"

	# This code expects to read mission_file.csv to be found two levels up from the file it is in
	# As of 04/14/2013, this code is stored in the folder 'programming_tools', and olinoboat is two levels up
	basepath = os.path.dirname(os.path.abspath(__file__))
	filepath = os.path.abspath(os.path.join(basepath, "..", "..", filename))

	mission_list = []
	
	try:
		# Reads the csv file and turns it into lists of numbers
		with open(filepath, 'rb') as csvfile:
			mission_reader = csv.reader(csvfile, delimiter=',')
			for row in mission_reader:
				row = [literal_eval(item.strip()) for item in row]
				mission_list.append(row)
				# print row
		interpreted_missions =  interpret_mission_file(mission_list)
		return interpreted_missions

	except IOError as e:
		if e.errno == 2:
			print "read_mission.py: I expected to find %s file at %s, but if it's not there" %(filename, filepath)
		else:
			print "IOError: " + str(e)
		return [0, 0, 0, 0]
