#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16
from hardware import sensors
from ast import literal_eval
from math import hypot

mission_goal=None

class MissionGoal():

    def __init__(self, sensors, waypoints, node = 0):
        self.callback = self.__default_callback
        self.sensors = sensors
        self.success_dis = 2    # Meters
        self.goals = waypoints
        self.index = 0
        self.current_goal = [0, 0]

        #initialize node if not running from the think code
        if node == 0:
            rospy.init_node('data_listener', anonymous=False)

        #set up subscribers to ("topic", DataType, callback_function)
        rospy.Subscriber("gps_lat", UInt16, self.__update_goal_point)
        rospy.loginfo("Mission goal publisher initialized")

    def __update_goal_point(self, data):
        
        boat_lat = sensors.gps_lat
        boat_lon = sensors.gps_lon
        rospy.loginfo("GPS sent signal: %i , %i" % (boat_lat, boat_lon))
        
        temp_goal = literal_eval(self.goals[index])
        target_dis = hypot((temp_goal[0]-boat_lat), (temp_goal[1]-boat_lon))

        if target_dis < self.success_dis:
            index += 1

        self.current_goal = literal_eval(self.goals[index])
        self.callback(self.current_goal)

    def set_callback(self,callback):
        self.callback = callback

    def __default_callback(self, data):
        rospy.loginfo("Current goal set to %s, but no additional callback has been registered" % data)

def init(node):
    global mission_goal
    sensors.init()

    # waypoints = yaml.yaml_parse(file)
    waypoints = ['[0, 0]', '[2, 2]', '[5, 5]']

    mission_goal = MissionGoal(0, sensors, waypoints, node)
 
if __name__ == '__main__':
   pass
