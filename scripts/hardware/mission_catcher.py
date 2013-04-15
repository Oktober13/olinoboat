#!/usr/bin/env python
import roslib; roslib.load_manifest('olinoboat')
import rospy
from std_msgs.msg import String
from ast import literal_eval

mission_goal=None

class MissionGoal():

    def __init__(self, node = 0):
        self.callback = self.__default_callback
        self.current_goal = [2, 251535]   #TODO change back to [0,0]
        #initialize node if not running from the think code
        if node == 0:
            rospy.init_node('data_listener', anonymous=False)

        # set up subscribers to ("topic", DataType, callback_function)
        rospy.Subscriber("current_waypoint_goal", String, self.__update_goal_point)
        rospy.loginfo("mission_catcher.py: Mission goal catcher initialized")

    def __update_goal_point(self, data):
        
        rospy.loginfo("mission_catcher.py: Saving goal point %s" %(data.data))
        self.current_goal = literal_eval(self.goals)[self.index]
        self.callback()

    def set_callback(self,callback):
        self.callback = callback

    def __default_callback(self, data):
        rospy.loginfo("mission_catcher.py: Current goal caught and set to %s, but no additional callback has been registered" % data)

def init(node):
    global mission_goal
    mission_goal = MissionGoal(node)

if __name__ == '__main__':
   pass
