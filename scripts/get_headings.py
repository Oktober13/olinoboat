import roslib; roslib.load_manifest('olinoboat')
import rospy
from ast import literal_eval
from std_msgs.msg import UInt16

class Heading():

    def __init__(self, heading_topics, node = 0):
    	self.__update_heading_after_number = len(heading_topics)
    	self.__headings_received = []
    	self.current_heading_array = [1]*360
    	self.__working_heading_array = [1]*360
        #initialize node if not running from the think code
        if node == 0:
            rospy.init_node('data_listener', anonymous=False)
        for topic in heading_topics:
        	rospy.Subscriber("topic", str, self.__save_heading)
        	rospy.loginfo("Now getting heading from" + topic)

    def __set_offset(self, data):
        rospy.loginfo("encoder send offset signal: %i" % (data.data))
        self.offset = data.data

    def self.__save_heading(self, data):
    	self.__headings_received += 1
    	node_name = ":".split(data.data)
        rospy.loginfo("received heading from source:%f" % (self.__headings_received))
        heading_array = literal_eval(data.data)
        self.__working_array = self.__working_array*heading_array
        if self.__headings_received > self.__update_headings_after_number:
        	self.current_heading_array = self.__working_array
        	self.__headings_received = 0

        

