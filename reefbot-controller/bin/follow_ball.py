#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
from reefbot_msgs.msg import RobotStatus
import time
import math
from robot_api import *

# set the *_max values below to control max values
# currenty left and right are scaled.
# lights and vertical are the actual values used
left_motor_max = 10.0 * 0.2
right_motor_max = 10.0 * 0.2
lights_max = 80.0
vertical_motor_max = 70.0

horiz_pixels = 540
vert_pixels = 410

left_motor_cmd = 0
right_motor_cmd = 0
vertical_motor_cmd = 0
lights_max_cmd = 0

last_ball_msg = Point()
last_robot_msg = RobotStatus()

heading_setpoint = 0
vert_setpoint = 0
forward_setpoint = 0

vert_kP = vertical_motor_max / 2.0  # max vertical thrust when 2 meters above / below
forward_kP = left_motor_max / 5.0  # max forward thrust when 5 meters away
heading_kP = left_motor_max / (3.14)  # max turning when 180 deg off

autonomous_mode = False

global left_motor_cmd, right_motor_cmd, vertical_motor_cmd, lights_max_cmd
global last_ball_msg, last_robot_msg

ball_msg_timeout = 1.0 # s
global ball_msg_received_time

def store_ball_info(data): # callback for ball detection
    last_ball_msg = data
    ball_msg_received_time = time.time()
    set_setpoints()


def get_ball_distance(radius):
    m_rad = -0.5 # tune these based on calibration
    b_rad = 2
    distance = radius * m_rad + b_rad # inverse linear relationship


def normalize_pixels(x_raw, y_raw):
    x_norm = (x_raw / horiz_pixels) * 2.0 - 1.0
    y_norm = (y_raw / vert_pixels) * 2.0 - 1.0
    return x_norm, y_norm


def set_setpoints():
    x_norm, y_norm = normalize_pixels(last_ball_msg.x, last_ball_msg.y)
    # set vertical set point
    # positive error means target is up
    vertical_error = y_norm
    vert_setpoint = vertical_error

    # set forward set point
    # positive error means target is forward
    actual_distance = get_ball_distance(last_ball_msg.z)
    desired_separation_distance = 1.0
    forward_setpoint = actual_distance - desired_separation_distance

    # set angle set point
    # positive error means target is to the right
    angle_error = math.atan2(x_norm, actual_distance)
    heading_setpoint = angle_error + last_robot_msg.heading


def store_robot_info(data):
    last_robot_msg = data


def stay_still():
    left_motor_cmd = 0
    right_motor_cmd = 0    
    vertical_motor_cmd = 0

    
def move_to_ball():
    left_motor_cmd = 0
    right_motor_cmd = 0

    # get errors
    forward_error = forward_setpoint  # we don't have sensor data on this
    heading_error = last_robot_msg.heading - heading_setpoint
    vert_error = vert_setpoint

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


def joy_callback(data):
    if buttons[0] > 0.9:
        print('Autonomous mode commanded')
        autonomous_mode = True
        lights_max_cmd = 20.0

    else:    
        print('Received joystick message') 
        
        left_motor_cmd = 0 
        right_motor_cmd = 0 
        vertical_motor_cmd = 0
           
        # Set right and left thruster command  
        # Set to use just one joystick. So other joystick can control vertical
        if (data.axes[1] > 0.1): # filter out really small values
                left_motor_cmd = int(data.axes[1] * left_motor_max * -.3)
        if (data.axes[1] < -0.1):
                left_motor_cmd = int(abs(data.axes[1]) * left_motor_max *.3)
        
        if (data.axes[4] > 0.1): # filter out really small values
            right_motor_cmd = int(data.axes[4] * right_motor_max * -.3)
        if (data.axes[4] < -0.1):
            right_motor_cmd = int(abs(data.axes[4]) * right_motor_max * .3)


        # now set the vertical control thruster
        if (data.axes[7] > 0.8):
            vertical_motor_cmd = vertical_motor_max
        if (data.axes[7] < -0.8):
            vertical_motor_cmd = -1 *vertical_motor_max
        
        if data.buttons[5] > .2:
            left_motor_cmd = int(left_motor_max * -1)
            right_motor_cmd = int(right_motor_max * -1)

        if data.buttons[4] > .2:
            left_motor_cmd = int(left_motor_max)
            right_motor_cmd = int(right_motor_max)

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
            lights_max_cmd = data.axes[5] * -.1*lights_max
            print lights_max_cmd


def within_timeout_period():
    if time.time()-ball_msg_received_time <ball_msg_timeout:
        return True
    else:
        return False
    
def listener(args):
    rospy.init_node('follow_ball')
    print "Starting Controller\n"
    # Connect to reefbot robot
    v = VideoRayAPI()
    v.connect('192.168.1.2', 100)

    # enable power for the onboard camera
    v.toggle_camera(True)
    v.send_command()

    rospy.Subscriber("ball_detections", Point, store_ball_info, queue_size=1)
    rospy.Subscriber("robot_status", RobotStatus, store_robot_info, queue_size=1)
    rospy.Subscriber("joy", Joy, joy_callback, queue_size=1)

    rate = rospy.Rate(5)  # in Hz
    while not rospy.is_shutdown():
        if autonomous_mode:
            print('In autonomous mode')
            if within_timeout_period(): # if you've received a recent ball detection message
                print("Spotted a ball!")
                move_to_ball() # this calculates the motor commands which will be sent in the following lines
            else:
                print("Where's that ball?")
                stay_still()
        v.set_velocity(left_motor_cmd, right_motor_cmd)
        v.set_vertical(vertical_motor_cmd)
        v.set_lights(lights_max_cmd)
        v.send_command()
        print "Sending Commands Left:", left_motor_cmd, " Right:", right_motor_cmd, "Vertical:", vertical_motor_cmd, "Lights:", lights_max_cmd, ".\n"

        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    listener(sys.argv)
