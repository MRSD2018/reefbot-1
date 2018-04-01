#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Joy
import time
# ROS Node converts Joystick inputs from the joy node
# Gets joystick messages to the Joy topic
# then converts the joystick inputs into Twist commands
# axes 1: left stick vertical controls linear speed
# axes 0: left stick horizontal controls angular speed
from robot_api import *

# set the *_max values below to control max values
# currenty left and right are scaled.
# lights and vertical are the actual values used
left_motor_max = 10.0
right_motor_max = 10.0
lights_max = 80.0
vertical_motor_max = 70.0
current_time = 14.8
command_time = 0.9

left_motor_cmd = 0
right_motor_cmd = 0
vertical_motor_cmd = 0
lights_max_cmd = 0

def joy_callback(data):

    start_time = time.time()

    print('Received joystick message') 
    #print data.axes 
    
    
    global left_motor_cmd, right_motor_cmd,vertical_motor_cmd,lights_max_cmd
    left_motor_cmd = 0 
    right_motor_cmd = 0 
    vertical_motor_cmd = 0
       
    # Set right and left thruster command  
    # Set to use just one joystick. So other joystick can control vertical
    if (data.axes[1] > 0.1): # filter out really small values
            left_motor_cmd = int(data.axes[1] * left_motor_max * -.7)
    if (data.axes[1] < -0.1):
            left_motor_cmd = int(abs(data.axes[1]) * left_motor_max *.7)
    
    if (data.axes[4] > 0.1): # filter out really small values
        right_motor_cmd = int(data.axes[4] * right_motor_max * -.7)
    if (data.axes[4] < -0.1):
        right_motor_cmd = int(abs(data.axes[4]) * right_motor_max * .7)


    # now set the vertical control thruster
    if (data.axes[7] > 0.8):
        vertical_motor_cmd = vertical_motor_max
    if (data.axes[7] < -0.8):
        vertical_motor_cmd = -1 *vertical_motor_max
    
    if data.buttons[5] > 0.5:
        left_motor_cmd = int(left_motor_max * -1) * 0.75
        right_motor_cmd = int(right_motor_max * -1) * 0.75

    if data.buttons[4] > 0.5:
        left_motor_cmd = int(left_motor_max) * 0.75
        right_motor_cmd = int(right_motor_max) * 0.75

    if data.buttons[1] > 0.5:
        left_motor_cmd = 0
        right_motor_cmd = 0
        lights_max_cmd = lights_max

    if data.buttons[0] > 0.5:
        left_motor_cmd = 0 
        right_motor_cmd = 0 
        lights_max_cmd = 0


    # Light control
    if data.axes[2] < -0.8:
        lights_max_cmd = data.axes[5] * -1 * lights_max
        print lights_max_cmd
    
        # if (data.axes[2] >-0.7):
        #         lights_max_cmd = data.axes[2] * lights_max
        # if (data.axes[5] < 0.7):
        #         lights_max_cmd = data.axes[5] * -1 * lights_max
    
    end_time = time.time()
    print "joy_callback", end_time-start_time

def listener(args):
    rospy.init_node('joy_listen')
    print "Starting Controller\n"       
    # Connect to reefbot robot  
    v = VideoRayAPI()
    v.connect('192.168.1.2',100)

    # enable power for the onboard camera
    v.toggle_camera(True)
    v.send_command()    

    rospy.Subscriber("joy",Joy,joy_callback, queue_size=1)

    rate = rospy.Rate(10) # in Hz
    while not rospy.is_shutdown():

        start_time = time.time()
        v.set_velocity(left_motor_cmd,right_motor_cmd)
        v.set_vertical(vertical_motor_cmd)
        v.set_lights(lights_max_cmd)
        v.send_command()
        end_time = time.time()
        print "send_commands", end_time-start_time
        print "Sending Commands Left:", left_motor_cmd, " Right:", right_motor_cmd, "Vertical:", vertical_motor_cmd, "Lights:", lights_max_cmd, ".\n"
        rate.sleep()
    
    rospy.spin()

if __name__ == '__main__':
    listener(sys.argv)

