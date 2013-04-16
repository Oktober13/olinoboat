#!/usr/bin/env python

# mission.py contains a class that is initialized by another piece of code
# mission.py is only currently (04/14/2013) only initialized by mission_publish.py
#
#       This class, MissionGoal, constantly checks the boat position against the list of missions the boat wants to accomplish
#            Whenever the boat gets within 'success_dis' meters (this is changeable below) the MissionGoal sets the next GPS Waypoint as the current goal
#       When it is first initialized, MissionGoal will read a file called mission_file.csv that sets the string of missions that boat should accomplish
#       More details on how to set up missions can be found in mission_file_csv_howto.txt in the olinoboat folder

# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import Float64, String
from math import hypot

# Imports necessary sailbot code
from hardware import sensors
from programming_tools import read_mission

mission_goal=None

class MissionGoal():

    def __init__(self, sensors, waypoints, node = 0):
        self.callback = self.__default_callback
        self.sensors = sensors
        self.success_dis = 2    # Meters
        self.goals = waypoints 
            # In original sailbot code, the string of waypoints never changes once it is set at the beginning
            # It would be possible to make the boat add or delete waypoints mid-run, depending on conditions, by editing the list self.goals
            # This could be very useful or very painful, depending on how you implement it
        self.index = 0
        self.current_goal = [0, 0]

        #initialize node if not running from the think code
        if node == 0:
            rospy.init_node('data_listener', anonymous=False)

        # set up subscribers to ("topic", DataType, callback_function)
        rospy.Subscriber("gps_lat", Float64, self.__update_goal_point)
        rospy.loginfo("mission.py: Mission goal publisher initialized")

    def __update_goal_point(self, data):
        rospy.loginfo("mission.py: Grabbing GPS data")
        boat_x = self.sensors.gps.current_x     # Data type: Float64
        boat_y = self.sensors.gps.current_y     # Data type: Float64

        rospy.loginfo("mission.py: Checking GPS signal x=%f , y=%f against the current goal" % (boat_x, boat_y))

        temp_goal = self.goals[self.index]
        target_dis = hypot((temp_goal[0]-boat_x), (temp_goal[1]-boat_y))

        if target_dis < self.success_dis:
            self.index += 1
            rospy.loginfo("mission.py: Hit waypoint! Got within %f meters" %target_dis)

        self.current_goal = self.goals[self.index]
        self.callback()

    def set_callback(self,callback):
        self.callback = callback

    def __default_callback(self, data):
        rospy.loginfo("mission.py: Current goal set to %s, but no additional callback has been registered" % data)

def init(node):
    global mission_goal
    sensors.init(node)

    # This reads the list of waypoint from a file called mission_file.csv in the folder olinoboat
    waypoints = read_mission.read_mission_csv()

    mission_goal = MissionGoal(sensors, waypoints, node)


if __name__ == '__main__':
   pass
