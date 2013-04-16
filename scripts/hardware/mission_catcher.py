#!/usr/bin/env python

# mission_catcher.py contains a class that needs to be initialized by another piece of code to use
# mission_catcher.py depends on (is subscribed to) mission_publish.py
# As of 04/14/2013, go_short.py is the only piece of code that calls mission_catcher.py
#
# The string of dependencies:
#      When new GPS data comes in, mission.py checks whether the boat has hit the waypoint and, if so, moves on to the next waypoint
#      When mission.py updates, mission_publish.py publishes whatever it output
#      Whenever mission_publish.py publishes, mission_catcher.py saves that data
#      Any function that initializes mission_catcher.py can now access the variable mission_goal.current_goal
#
# The architecture was structured with these dependencies so that the user can intialize mission_catcher.py instead of mission.py
# This is simpler for the computer, because mission.py involves more computation than mission_catcher.py
# As of 04/14/2013, mission_publish is the only thing initializing mission.py, and every other piece of code that needs the current waypoint goal can initialize mission_catcher.py


# Imports necessary libraries
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import String
from ast import literal_eval

mission_goal=None

class MissionGoal():

    def __init__(self, node = 0):
        self.current_goal = [0,0]
        self.callback = self.__default_callback
        
        #initialize node if not running from the think code
        if node == 0:
            rospy.init_node('data_listener', anonymous=False)

        # set up subscribers to ("topic", DataType, callback_function)
        rospy.Subscriber("current_waypoint_goal", String, self.__update_goal_point)
        rospy.loginfo("mission_catcher.py: Mission goal catcher initialized")

    def __update_goal_point(self, data):
        
        rospy.loginfo("mission_catcher.py: Recieved goal point %s" %(data.data))

        self.current_goal = literal_eval(data.data)
        self.callback()

    def set_callback(self,callback):
        self.callback = callback

    def __default_callback(self):
        rospy.loginfo("mission_catcher.py: Current goal caught and set to %s, but no additional callback has been registered" % self.current_goal)

def init(node):
    global mission_goal
    mission_goal = MissionGoal(node)

if __name__ == '__main__':
   pass
