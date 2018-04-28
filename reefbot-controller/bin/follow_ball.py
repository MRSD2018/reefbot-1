#!/usr/bin/env python

import roslib; roslib.load_manifest('reefbot-controller')
import rospy
import sys
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
from reefbot_msgs.msg import RobotStatus
import time
import math
from robot_api import *
import numpy as np

# set the *_max values below to control max values
# currenty left and right are scaled.
# lights and vertical are the actual values used
# left_motor_max = 0 # 10.0 * 0.2
# right_motor_max = 0 #10.0 * 0.2
# lights_max = 80.0
# vertical_motor_max = 0 # 70.0 * 0.05

left_motor_cmd = 0
right_motor_cmd = 0
vertical_motor_cmd = 0
lights_max_cmd = 0

detection_data = Point()

autonomous_mode = False

max_cmds = {'left': 2.0, 'right': 2.0, 'vert': 3.5, 'lights': 8.0}


class Controller:
    def __init__(self, max_cmds):
        self.max_motor_cmds = {'left': max_cmds['left'],  \
                               'right': max_cmds['right'],  \
                               'vert': max_cmds['vert']}
        self.motor_cmds =   {'left': 0.0,   'right': 0.0,   'vert': 0.0}
        self.ball =         {'cx': 0.0,     'cy': 0.0,      'dist': 0.0} # ball position and size in frame
        self.update =       {'cx': 0.0,     'cy': 0.0,      'dist': 0.0} # ball position and size in frame
        self.gains =        {'fwd': 1.0,    'yaw': 1.0,     'vert': 1.0}
        self.errors =       {'fwd': 0.0,    'yaw': 0.0,     'vert': 0.0}
        self.desired_separation_distance = 1.0 # m
        self.horiz_pixels = 540.0
        self.vert_pixels = 410.0
        self.gamma = 0.8
        # self.ball_av = {'cx': np.zeros(1),     'cy': np.zeros(1),      'dist': np.zeros(1)}
        # self.num_hist_elms = 5

    def get_motor_cmds(self):
        return self.motor_cmds['left'], self.motor_cmds['right'], self.motor_cmds['vert']


    def get_ball_distance(self, radius):
        return 1/((radius-5.69)/61.2) # meters


    def normalize_pixels(self, x_raw, y_raw):
        x_norm = (x_raw / self.horiz_pixels) * 2.0 - 1.0
        y_norm = (y_raw / self.vert_pixels) * 2.0 - 1.0
        return x_norm, y_norm


    def ball_visible(self, data):
        if data.x < 0 or data.y < 0 or data.z < 0: # corresponds to no detection 
            return False
        else:
            return True


    def stay_still(self):
        self.motor_cmds = self.motor_cmds.fromkeys(self.motor_cmds, 0.0) # reset all to 0


    def update_states(self, data):
        # update ball location
        self.update['cx'], self.update['cy'] = self.normalize_pixels(data.x, data.y)
        self.update['dist']                  = self.get_ball_distance(data.z)

        self.ball['cx']                      = (1.0 - self.gamma) * self.ball['cx'] + self.gamma * self.update['cx']
        self.ball['cy']                      = (1.0 - self.gamma) * self.ball['cy'] + self.gamma * self.update['cy']
        self.ball['dist']                    = (1.0 - self.gamma) * self.ball['dist'] + self.gamma * self.update['dist']
 

    def move(self, data, current_heading):
        self.update_states(data)

        # get errors
        self.errors['fwd']  =   self.ball['dist'] - self.desired_separation_distance
        self.errors['yaw']  =   math.atan2(self.ball['cx'], self.ball['dist'])
        self.errors['vert'] =   self.ball['cy']

        # set motor commands
        self.motor_cmds['vert']     = self.gains['vert'] * self.errors['vert']
        self.motor_cmds['left']     = self.gains['fwd'] * self.errors['fwd'] + self.gains['yaw'] * self.errors['yaw']
        self.motor_cmds['right']    = self.gains['fwd'] * self.errors['fwd'] - self.gains['yaw'] * self.errors['yaw']

        # check against max
        # self.motor_cmds['vert']     = min(self.max_motor_cmds['vert'], self.motor_cmds['vert'])# * np.sign(self.motor_cmds['vert'])
        # self.motor_cmds['left']     = min(self.max_motor_cmds['left'], self.motor_cmds['left'])# * np.sign(self.motor_cmds['left'])
        # self.motor_cmds['right']    = min(self.max_motor_cmds['right'], self.motor_cmds['right'])# * np.sign(self.motor_cmds['right'])



def process_ball_detection_msg(data): # callback for ball detection
    global detection_data
    detection_data = data


def joy_callback(data):
    global left_motor_cmd, right_motor_cmd, vertical_motor_cmd, lights_max_cmd
    global max_cmds

    if data.buttons[0] > 0.9:
        # print('Autonomous mode commanded')
        global autonomous_mode
        autonomous_mode = True
        lights_max_cmd = max_cmds['lights'] * 0.5

    else:    
        autonomous_mode = False
        # print('Received joystick message') 
        
        # left_motor_cmd = 0 
        # right_motor_cmd = 0 
        # vertical_motor_cmd = 0
        # lights_max_cmd = 0

        # map joystick inputs
        left_motor_input = -data.axes[1] # continuous in range of [-1, 1]
        right_motor_input = -data.axes[4] # continuous in range of [-1, 1]
        vertical_motor_input = 1.0-(data.axes[2]+1.0)/2.0 # continuous in range of [0, 1]
        vertical_direction = data.buttons[4] # 0 when NOT pressed, 1 when pressed
        if vertical_direction == 1: # going UP 
            vertical_direction_cmd = -1.0
        if vertical_direction == 0: # going DOWN
            vertical_direction_cmd = 1.0
        main_thrust_reduction_factor = 1.0 # 0.2
        vertical_thrust_reduction_factor = 1.0 # 0.05

        left_motor_cmd = left_motor_input * max_cmds['left']
        right_motor_cmd = right_motor_input * max_cmds['right']
        vertical_motor_cmd = vertical_motor_input * max_cmds['vert'] * vertical_direction_cmd



def listener(args):
    rospy.init_node('follow_ball')
    print "Starting Controller\n"
    # Connect to reefbot robot
    v = VideoRayAPI()
    v.connect('192.168.1.2', 100)

    # enable power for the onboard camera
    v.toggle_camera(True)
    v.send_command()

    rospy.Subscriber("ball_detection", Point, process_ball_detection_msg, queue_size=1)
    rospy.Subscriber("joy", Joy, joy_callback, queue_size=1)

    global left_motor_cmd, right_motor_cmd, vertical_motor_cmd, lights_max_cmd
    global max_cmds
    c = Controller(max_cmds)

    rate = rospy.Rate(5)  # in Hz
    while not rospy.is_shutdown():
        if autonomous_mode:
            if c.ball_visible(detection_data):
                c.move(detection_data, v.robotStatus.heading) # this calculates the motor commands which will be sent in the following lines
            else:
                c.stay_still()
            left_motor_cmd, right_motor_cmd, vertical_motor_cmd = c.get_motor_cmds()
            # print c.ball
        # left_motor_cmd = 0
        v.set_velocity(left_motor_cmd, right_motor_cmd)
        v.set_vertical(vertical_motor_cmd)
        # v.set_lights(lights_max_cmd)
        v.send_command()
        print "Autonomous: ", autonomous_mode
        print "Sending Commands Left:", left_motor_cmd, " Right:", right_motor_cmd, "Vertical:", vertical_motor_cmd, "Lights:", lights_max_cmd, ".\n"

        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    listener(sys.argv)
