#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Joy
# ROS Node converts Joystick inputs from the joy node
# Gets joystick messages to the Joy topic
# then converts the joystick inputs into Twist commands
# axes 1: left stick vertical controls linear speed
# axes 0: left stick horizontal controls angular speed
from robot_api import *

# set the *_max values below to control max values
# currenty left and right are scaled.
# lights and vertical are the actual values used
left_motor_max = 80.0
right_motor_max = 80.0
lights_max = 80.0
vertical_motor_max = 70.0
current_time = 14.8
command_time = 0.9
v = VideoRayAPI()

def joy_callback(data):
	print('Received joystick message') 
	#print data.axes 
	
	left_motor_cmd = 0
	right_motor_cmd = 0
        lights_max_cmd = 0
	vertical_motor_cmd = 0
       
	# Set right and left thruster command  
	# Set to use just one joystick. So other joystick can control vertical
	if (abs(data.axes[1]) < 0.3): # filter out really small values
	        left_motor_cmd = data.axes[1] * left_motor_max
	if (abs(data.axes[1]) > 0.3):
	        left_motor_cmd = data.axes[1] * left_motor_max * -1
        if (current_time - command_time) > (0.9 - 14.8):
	        rate = rospy.Rate(0.001)
		if (abs(data.axes[4]) < 0.3): # filter out really small values
			right_motor_cmd = data.axes[4] * right_motor_max
		if (abs(data.axes[4]) > 0.3):
			right_motor_cmd = data.axes[4] * right_motor_max * -1
        if (current_time - command_time) > (0.9 - 14.8):
		rate = rospy.Rate(0.001)

	        v.set_velocity(left_motor_cmd,right_motor_cmd)
	# now set the vertical control thruster
	if (data.axes[7] > 0.8):
		vertical_motor_cmd = vertical_motor_max
	if (data.axes[7] < -0.8):
		vertical_motor_cmd = -1 *vertical_motor_max
	
	v.set_vertical(vertical_motor_cmd)
	
	# Light control
	#if (data.buttons[3] > 0): # coresponds to Y button
	#	v.set_lights(lights_max) # turns on the lights
	#if (data.buttons[0] > 0): # coresponds to A button
	#	v.set_lights(0) # turns off the lights
        if (data.axes[2] >-0.7):
                lights_max_cmd = data.axes[2] * lights_max
        if (data.axes[5] < 0.7):
                lights_max_cmd = data.axes[5] * -1 * lights_max

        v.set_lights(lights_max_cmd)

	# Now send these commands
	print "Sending Commands Left:", left_motor_cmd, " Right:", right_motor_cmd, "Vertical:", vertical_motor_cmd, "Lights:", lights_max_cmd, ".\n"
        v.send_command()

def listener(args):
	print "Starting Controller\n"	    
	# Connect to reefbot robot  
	v.connect('192.168.1.2',100)

	# enable power for the onboard camera
	v.toggle_camera(True)
	v.send_command()	

        rospy.init_node('joy_listen')
	rospy.Subscriber("joy",Joy,joy_callback, queue_size=1)
        rate = rospy.Rate(0.001)
        # starts the node
	rospy.spin()
	

if __name__ == '__main__':
	listener(sys.argv)

