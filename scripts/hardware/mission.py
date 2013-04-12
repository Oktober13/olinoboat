#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import UInt16
from hardware import sensors

mission_goal=None
waypoints = yaml.yaml_parse(file)
index = 0

class MissionGoal():

    def __init__(self, sensors, node = 0):
        self.latitude = 0
        self.longitude = 0
        self.goal = '[0, 0]'
        self.callback = self.__default_callback
        self.sensors = sensors


        #initialize node if not running from the think code
        if node == 0:
            rospy.init_node('data_listener', anonymous=False)

        #set up subscribers to ("topic", DataType, callback_function)
        rospy.Subscriber("gps_lat", UInt16, self.__update_goal_point)
        rospy.Subscriber("gps_lon", UInt16, self.__update_goal_point)
        # I DON"T KNOW HOW TO DEAL WITH SULTIPLE SUBSCRIBERS....
        rospy.loginfo("Mission goal publisher initialized")

    def __update_goal_point(self, data):
        # HOW CAN WE MAKE THIS DEAL WITH MULTIPE SUBSCRIBERS?
        rospy.loginfo("GPS sent signal: %i , %i" % (data.data), (data.data))
        
        # This is what you do with the GPS point
        current_goal = get_goal(file)
        if point is close to current goal:
            move_to_next_mission(file)
        publish_current_goal(file)
        
        self.callback()

    def set_callback(self,callback):
        self.callback = callback

    def __default_callback(self):
        rospy.loginfo("Current goal set to %s, but no additional callback has been registered" % (self.goal))

def init(node):
    global mission_goal
    sensors.init()
    mission_goal = MissionGoal(0, sensors, node)
 
if __name__ == '__main__':
   pass
