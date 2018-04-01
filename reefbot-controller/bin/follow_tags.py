#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Joy
from apriltags_ros.msg import AprilTagDetection
from reefbot_msgs.msg import RobotStatus
import time
import math
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

last_tag_msg = AprilTagDetection()
last_robot_msg = RobotStatus()

heading_setpoint = 0
vert_setpoint = 0
forward_setpoint = 0

vert_kP = vertical_motor_max / 2.0  # max vertical thrust when 2 meters above / below
forward_kP = left_motor_max / 5.0  # max forward thrust when 5 meters away
heading_kP = left_motor_max / (3.14)  # max turning when 180 deg off


def store_tag_info(data):
    global last_tag_msg
    last_tag_msg = data
    set_setpoints()


def set_setpoints():
    global vert_setpoint, heading_setpoint, forward_setpoint
    # set vertical set point
    # positive error means target is up
    vertical_error = -last_tag_msg.pose.pose.position.y
    vert_setpoint = vertical_error + last_robot_msg.depth

    # set forward set point
    # positive error means target is forward
    distance_error = last_tag_msg.pose.pose.position.z
    forward_setpoint = distance_error

    # set angle set point
    # positive error means target is to the right
    angle_error = math.atan2(last_tag_msg.pose.pose.position.x, last_tag_msg.pose.pose.position.z)
    heading_setpoint = angle_error + last_robot_msg.heading


def store_robot_info(data):
    global last_robot_msg
    last_robot_msg = data


def move_to_tag():
    global left_motor_cmd, right_motor_cmd, vertical_motor_cmd
    left_motor_cmd = 0
    right_motor_cmd = 0

    # get errors
    forward_error = forward_setpoint  # we don't have sensor data on this
    heading_error = last_robot_msg.heading - heading_setpoint
    vert_error = last_robot_msg.depth - vert_setpoint

    # move forward
    left_motor_cmd += forward_kP * forward_error
    right_motor_cmd += forward_kP * forward_error

    # turn
    left_motor_cmd += heading_kP * heading_error
    right_motor_cmd += -heading_kP * heading_error

    # get it normalized
    if left_motor_cmd > left_motor_max or right_motor_cmd > right_motor_max:
        mult_factor = left_motor_max / max(left_motor_cmd, right_motor_cmd)
        left_motor_cmd *= mult_factor
        right_motor_cmd *= mult_factor

    # go up or down
    vertical_motor_cmd = vert_kP * vert_error


def listener(args):
    rospy.init_node('follow_tags')
    print "Starting Controller\n"
    # Connect to reefbot robot
    v = VideoRayAPI()
    v.connect('192.168.1.2', 100)

    # enable power for the onboard camera
    v.toggle_camera(True)
    v.send_command()

    rospy.Subscriber("tag_detections", AprilTagDetection, store_tag_info, queue_size=1)
    rospy.Subscriber("robot_status", RobotStatus, store_robot_info, queue_size=1)

    rate = rospy.Rate(5)  # in Hz
    while not rospy.is_shutdown():
        v.set_velocity(left_motor_cmd, right_motor_cmd)
        v.set_vertical(vertical_motor_cmd)
        v.set_lights(lights_max_cmd)
        v.send_command()
        print "Sending Commands Left:", left_motor_cmd, " Right:", right_motor_cmd, "Vertical:", vertical_motor_cmd, "Lights:", lights_max_cmd, ".\n"

        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    listener(sys.argv)
