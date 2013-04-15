#!/usr/bin/env python
from pylab import *

from programming_tools import read_mission

missions = read_mission.read_mission_csv()
print missions

# plots the waypoints
for item in missions:
	plot(item[0], item[1], 'yo')

# prints the last waypoint as blue
plot(missions[-1][0], missions[-1][1], 'bo')

show()