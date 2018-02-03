#!/usr/bin/python
from robot_api import *
import sys
import time

left_motor = 0
right_motor = 0

if len(sys.argv) >= 2:
	left_motor = float(sys.argv[1])
if len(sys.argv) >= 3:
	right_motor = float(sys.argv[2])

v = VideoRayAPI()
v.connect('192.168.1.2',100)

v.set_velocity(left_motor,right_motor)
#print v.send_command()
v.send_command()

time.sleep(2)

v.set_velocity(0,0)
v.send_command()
time.sleep(1)
