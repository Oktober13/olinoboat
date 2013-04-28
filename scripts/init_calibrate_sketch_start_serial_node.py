#!/usr/bin/env python

import os

#Change the calibrate sketch path to the right one for you computer
#Change the device name to the correct one for your setup. The Alamode on the raspberry pi is ttyS0. 
	#The Uno from the pi is ttyS1. The Uno from a laptop could be ttyACM0 or ttyAMA0 or ttyUSB0 or something else.
	#run "ls /dev" from the command line while the device is unplugged. Then plug it in and run "ls /dev" again. The new one is your device.

calibrate_sketch_path = '/home/trevor/groovy_workspace/sandbox/olinoboat/arduino/Calibrate'
device = 'ttyACM0'
arduino = 'uno'

def upload_arduino_sketch():
	os.chdir('%s' %calibrate_sketch_path)
	os.system('ino build -m %s' %arduino)
	os.system('ino upload -m %s -p /dev/%s' %(arduino, device))

def run_serial_node():
	print 'serial node going up'
	os.system('rosrun rosserial_python serial_node.py /dev/%s' %device)

if __name__ == "__main__":
	upload_arduino_sketch()
	run_serial_node()
	
