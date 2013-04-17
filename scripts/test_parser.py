#!/usr/bin/env python

# This code tests the sailbot's mission parser, read_mission.py
# When you run test_parser.py, this code reads the waypoints in mission_list.csv
# You can see the waypoints that the boat will try to execute, because the think code also runs read_mission.read_mission_csv(

# Imports necessary libraries
import numpy as np
import matplotlib.pyplot as plt

# Imports necessary sailbot code
from programming_tools import read_mission

missions = read_mission.read_mission_csv()
print missions

x = []
y = []
N = len(missions)
for i in range(N):
    x.append(missions[i][0])
    y.append(missions[i][1])

labels = ['waypoint{0}'.format(i) for i in range(N)]
plt.subplots_adjust(bottom = 0.1)
plt.scatter(x, y, marker = '^', s = 500)
for label, x, y in zip(labels, x, y):
    plt.annotate(
        label, 
        xy = (x, y), xytext = (-20, 20),
        textcoords = 'offset points', ha = 'right', va = 'bottom',
        bbox = dict(boxstyle = 'round,pad=0.5', fc = 'yellow', alpha = 0.5),
        arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3,rad=0'))

plt.show()