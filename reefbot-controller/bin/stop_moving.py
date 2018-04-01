#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Joy
from apriltags_ros.msg import AprilTagDetection
import time
import math
# ROS Node converts Joystick inputs from the joy node
# Gets joystick messages to the Joy topic
# then converts the joystick inputs into Twist commands
# axes 1: left stick vertical controls linear speed
# axes 0: left stick horizontal controls angular speed
from robot_api import *

left_motor_cmd = 0
right_motor_cmd = 0
vertical_motor_cmd = 0
lights_max_cmd = 0


def listener(args):
    rospy.init_node('follow_tags')
    print "Starting Controller\n"
    # Connect to reefbot robot

    v = VideoRayAPI()
    v.connect('192.168.1.2', 100)

    # enable power for the onboard camera
    v.toggle_camera(True)
    v.set_velocity(left_motor_cmd, right_motor_cmd)
    v.set_vertical(vertical_motor_cmd)
    v.set_lights(lights_max_cmd)
    v.send_command()


if __name__ == '__main__':
    listener(sys.argv)
