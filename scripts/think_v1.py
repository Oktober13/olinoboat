from think_behaviors import *
import think_behaviors
from hardware import servos, sensors
import rospy

current_node = rospy.init_node("think_v1",anonymous=True)
sensors.init(current_node)
servos.init(current_node)

rospy.Rate(10)

'''
servos:
	sail - servos.sail.set_position(angle)
	rudder - servos.sail.set_position(angle)

sensors: 
	compass - sensors.compass.angle
	gps - sensors.gps.current_location
	wind_angle - sensors.wind_angle.angle
	leak_detector - sensors.leak_detector.leak_callback = callback_function
					sensors.leak_detector.leak_detected
'''

loop_count = 0
next_tack = 100
goal = [76.2,42.1]
behaviors = think_behaviors.__all__
what_to_do = []

while not rospy.is_shutdown():
	for behavior in behaviors:
		current_behavior = getattr(think_behaviors,behavior)
		what_to_do.append(current_behavior.arbit(sensors,goal))
	# go_fast = go_fast(sensors,goal)
	# go_closest = go_closest(sensors)
	# spin_in_circle = spin_in_circle(sensors)
	# put_out_fire = put_out_fire()
	#["behavior",factor,do_the_thing()]

	Weight_things = {"tack",}
	rospy.sleep()	




	# if loop_count > next_tack:
	# 	print("tacking")
	# 	servos.rudder.set_position(-1*servos.rudder.current_position)
	# 	servos.sail.set_position(-1*servos.sail.current_position)
	# else:	
	# 	servos.sail.set_position(sensors.wind_angle.angle/2)

