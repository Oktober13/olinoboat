import curses
import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
from std_msgs.msg import UInt16

class teleop():
	def __init__(self, node = 0):
		self.rudder_angle = 90
		self.sail_angle = 90
		self.incr = 1
		self.set_publishers(node)
		self.verbose = 0


	def set_publishers(self, node):
		if node == 0:
			rospy.init_node('encoder_offset_setter')
		self.rudder_publisher = rospy.Publisher("rudder_servo", UInt16, latch = True)
		self.sail_publisher = rospy.Publisher("sail_servo", UInt16, latch = True)
    	
	def get_user_input(self):
		stdscr = curses.initscr()
		curses.cbreak()
		stdscr.keypad(1)
	 	
		if self.verbose:
			stdscr.addstr(0,10,"Hit 'q' to quit")
		stdscr.refresh()
	
		key = ''
		while key != ord('q'):
			key = stdscr.getch()
			if self.verbose:
				stdscr.addch(20,25,key)
			stdscr.refresh()
			if key == curses.KEY_UP: 
				if self.verbose:
					stdscr.addstr(2, 20, "Sail In")
					print "     sail in      "
				self.sail_angle -= self.incr
			elif key == curses.KEY_LEFT: 	
				if self.verbose:
					stdscr.addstr(3, 10, "Rudder Left")
					print "     rudder left  "
				self.rudder_angle -= self.incr
			elif key == curses.KEY_DOWN: 	
				if self.verbose: 
					stdscr.addstr(4, 20, "Sail Out")
					print "     sail out     "
				self.sail_angle += self.incr
			elif key == curses.KEY_RIGHT: 	
				if self.verbose: 
					stdscr.addstr(3, 30, "Rudder Right")
					print "     rudder right "
				self.rudder_angle += self.incr
	 	
			if self.verbose:		
				print "   rudder %f, sail %f" %(self.rudder_angle, self.sail_angle)
		
			if self.rudder_angle < 0:
				self.rudder_angle = 0
			if self.sail_angle < 0:
				self.sail_angle = 0
			self.rudder_publisher.publish(self.rudder_angle)
			self.sail_publisher.publish(self.sail_angle)
	    

		curses.endwin()
		return (self.rudder_angle, self.sail_angle)


if __name__ == "__main__":
	t=teleop()
	t.verbose = 1
	t.get_user_input()
	rospy.spin()
