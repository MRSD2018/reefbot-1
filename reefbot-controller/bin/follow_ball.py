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

# set the *_max values below to control max values
# currenty left and right are scaled.
# lights and vertical are the actual values used
left_motor_max = 10.0 * 0.2
right_motor_max = 10.0 * 0.2
lights_max = 80.0
vertical_motor_max = 70.0 * 0.05

# horiz_pixels = 540
# vert_pixels = 410

global last_ball_msg

left_motor_cmd = 0
right_motor_cmd = 0
vertical_motor_cmd = 0
lights_max_cmd = 0

detection_data = Point()

# heading_setpoint = 0
# vert_setpoint = 0
# forward_setpoint = 0

# vert_kP = vertical_motor_max / 2.0  # max vertical thrust when 2 meters above / below
# forward_kP = left_motor_max / 5.0  # max forward thrust when 5 meters away
# heading_kP = left_motor_max / (3.14)  # max turning when 180 deg off

autonomous_mode = False



# ball_msg_timeout = 1.0 # s
global ball_msg_received_time

class Controller:
    def __init__(self, max_motor_cmds):
        self.max_motor_cmds = {'left': max_motor_cmds['left'],  \
                               'right': max_motor_cmds['right'],  \
                               'vert': max_motor_cmds['vert']}
        self.motor_cmds =   {'left': 0.0,   'right': 0.0,   'vert': 0.0}
        self.ball =         {'cx': 0.0,     'cy': 0.0,      'dist': 0.0} # ball position and size in frame
        self.gains =        {'fwd': 1.0,    'yaw': 1.0,     'vert': 1.0}
        self.errors =       {'fwd': 0.0,    'yaw': 0.0,     'vert': 0.0}
        self.desired_separation_distance = 1.0 # m

        # self.setpoints = {'fwd': desired_separation_distance, 'yaw': 0.0, 'vert' = 0.0}
        self.horiz_pixels = 540.0
        self.vert_pixels = 410.0

    def get_motor_cmds(self):
        return self.motor_cmds['left'], self.motor_cmds['right'], self.motor_cmds['vert']

    def process_detection_data(self, detection_msg):
        self.ball['cx'], self.ball['cy'] = self.normalize_pixels(detection_msg.x, detection_msg.y)
        self.ball['dist'] = self.get_ball_distance(detection_msg.z)
        # self.set_setpoints()


    def get_ball_distance(self, radius):
        return 1/((radius-5.69)/61.2) # meters


    def normalize_pixels(self, x_raw, y_raw):
        x_norm = (x_raw / self.horiz_pixels) * 2.0 - 1.0
        y_norm = (y_raw / self.vert_pixels) * 2.0 - 1.0
        return x_norm, y_norm


    def stay_still():
        self.motor_cmds = self.motor_cmds.fromkeys(self.motor_cmds, 0.0) # reset all to 0

    def update(self, data):
        if data.x < 0 or data.y < 0 or data.z < 0: # corresponds to no detection 
            self.ball['cx']     =   0.0
            self.ball['cy']     =   0.0
            self.ball['dist']   =   self.desired_separation_distance
        else: 
            self.ball['cx']     =   data.x
            self.ball['cy']     =   data.y
            self.ball['dist']   =   data.z      

    def move(self, current_heading):
        # get errors
        self.errors['fwd']  =   self.ball['dist'] - self.desired_separation_distance
        self.errors['yaw']  =   math.atan2(self.ball['cx'], self.ball['dist'])
        self.errors['vert'] =   self.ball['cy']

        # set motor commands
        self.motor_cmds['vert'] = self.gains['vert'] * self.errors['vert']
        self.motor_cmds['left'] = self.gains['fwd'] * self.errors['fwd'] + self.gains['yaw'] * self.errors['yaw']
        self.motor_cmds['right'] = self.gains['fwd'] * self.errors['fwd'] - self.gains['yaw'] * self.errors['yaw']

        # check against max
        if left_motor_cmd > left_motor_max or right_motor_cmd > right_motor_max:
            mult_factor = left_motor_max / max(left_motor_cmd, right_motor_cmd)
            left_motor_cmd *= mult_factor
            right_motor_cmd *= mult_factor



def process_ball_detection_msg(data): # callback for ball detection
    detection_data = data


def joy_callback(data):
    if data.buttons[0] > 0.9:
        # print('Autonomous mode commanded')
        global autonomous_mode
        autonomous_mode = True
        lights_max_cmd = 5

    else:    
        autonomous_mode = False
        # print('Received joystick message') 
        
        left_motor_cmd = 0 
        right_motor_cmd = 0 
        vertical_motor_cmd = 0
        lights_max_cmd = 0

        # map joystick inputs
        left_motor_input = data.axes[1] # continuous in range of [-1, 1]
        right_motor_input = data.axes[4] # continuous in range of [-1, 1]
        vertical_motor_input = 1.0-(data.axes[2]+1.0)/2.0 # continuous in range of [0, 1]
        vertical_direction = data.buttons[4] # 0 when NOT pressed, 1 when pressed
        if vertical_direction == 1: # going UP 
            vertical_direction_cmd = -1.0
        if vertical_direction == 0: # going DOWN
            vertical_direction_cmd = 1.0
        main_thrust_reduction_factor = 1.0 # 0.2
        vertical_thrust_reduction_factor = 1.0 # 0.05

        left_motor_cmd = left_motor_input * main_thrust_reduction_factor * left_motor_max
        right_motor_cmd = right_motor_input * main_thrust_reduction_factor * right_motor_max
        vertical_motor_cmd = vertical_motor_input * vertical_thrust_reduction_factor * vertical_motor_max * vertical_direction_cmd

        global left_motor_cmd, right_motor_cmd, vertical_motor_cmd, lights_max_cmd


# def within_timeout_period():
#     if time.time()-ball_msg_received_time <ball_msg_timeout:
#         return True
#     else:
#         return False
    
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

    max_motor_cmds = {'left': 2.0, 'right': 2.0, 'vert': 3.5}

    c = Controller(max_motor_cmds)

    rate = rospy.Rate(5)  # in Hz
    while not rospy.is_shutdown():
        if autonomous_mode:
            # print('In autonomous mode')
        #     if within_timeout_period(): # if you've received a recent ball detection message
        #         print("Spotted a ball!")
            c.update(detection_data)
            c.move(v.robotStatus.heading) # this calculates the motor commands which will be sent in the following lines
            left_motor_cmd, right_motor_cmd, vertical_motor_cmd = c.get_motor_cmds()
        #     else:
        #         print("Where's that ball?")
        #         stay_still()
        # print left_motor_cmd, right_motor_cmd
        v.set_velocity(left_motor_cmd, right_motor_cmd)
        v.set_vertical(vertical_motor_cmd)
        # global robot_heading
        # robot_heading = v.robotStatus.heading
        v.set_lights(lights_max_cmd)
        v.send_command()
        print "Autonomous: ", autonomous_mode
        # print "Sending Commands Left:", left_motor_cmd, " Right:", right_motor_cmd, "Vertical:", vertical_motor_cmd, "Lights:", lights_max_cmd, ".\n"

        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    listener(sys.argv)
