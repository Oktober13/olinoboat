#!/usr/bin/env python
import csv
import os.path
from ast import literal_eval
from programming_tools import latlon_UTM

# def interpret_mission_file(mission_list):
# 	boat_goal_points = []
# 	for mission in mission_list:	# The format for a mission should be ['type of mission', 'information passing parameter', 'goal_latitude', 'goal_longitude']
# 		if mission[0] == 1:		# 'type of mission' = 1 means d owaypoint following
# 			boat_goal_points.append(mission[2], mission[3])

def read_mission_csv():
	# This code expects mission_file.csv to be found in the olinoboat folder
	filename = "mission_file.csv"

	basepath = os.path.dirname(os.path.abspath(__file__))
	filepath = os.path.abspath(os.path.join(basepath, "..", "..", filename))

	mission_list = []
	
	try:
		with open(filepath, 'rb') as csvfile:
			mission_reader = csv.reader(csvfile, delimiter=',')
			for row in mission_reader:
				row = [literal_eval(item.strip()) for item in row]
				mission_list.append(row)
				print row
		interpreted_missions =  interpret_mission_file(mission_list)
		return interpreted_missions

	except IOError as e:
		if e.errno == 2:
			print "read_mission.py: I expected to find %s file at %s, but if it's not there" %(filename, filepath)
		else:
			print "IOError: " + str(e)
		return [0, 0, 0, 0]
