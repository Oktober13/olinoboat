#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import Float64, String
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

        # set up subscribers to ("topic", DataType, callback_function)
        rospy.Subscriber("gps_lat", Float64, self.__update_goal_point)
        rospy.loginfo("mission.py: Mission goal publisher initialized")

    def __update_goal_point(self, data):
        
	rospy.loginfo("updating gps")
        boat_x = self.sensors.gps.current_x     # Data type: Float64
        boat_y = self.sensors.gps.current_y     # Data type: Float64

        rospy.loginfo("mission.py: Checking GPS signal x=%f , y=%f against the current goal" % (boat_x, boat_y))
        
        temp_goal = literal_eval(self.goals)[self.index]
        target_dis = hypot((temp_goal[0]-boat_x), (temp_goal[1]-boat_y))

        if target_dis < self.success_dis:
            self.index += 1
            rospy.loginfo("mission.py: Hit waypoint! Got within %f meters" %target_dis)

        self.current_goal = literal_eval(self.goals)[self.index]
        self.callback()

    def set_callback(self,callback):
        self.callback = callback

    def __default_callback(self, data):
        rospy.loginfo("mission.py: Current goal set to %s, but no additional callback has been registered" % data)

def init(node):
    global mission_goal
    sensors.init(node)

    # waypoints = yaml.yaml_parse(file)
    waypoints = '[[2 , 251537], [2, 2], [5, 5]]'

    mission_goal = MissionGoal(sensors, waypoints, node)




if __name__ == '__main__':
   pass
