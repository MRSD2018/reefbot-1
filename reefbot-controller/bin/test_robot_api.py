#!/usr/bin/python
from robot_api import *
import sys

left_motor = 10
right_motor = 10
lights = 10
vertical_motor = 10
if len(sys.argv) >= 2:
	left_motor = float(sys.argv[1])
if len(sys.argv) >= 3:
	right_motor = float(sys.argv[2])
if len(sys.argv) >= 4:
	lights = int(sys.argv[3])
if len(sys.argv) >= 5:
        vertical_motor = float(sys.argv[4])

v = VideoRayAPI()
v.connect('192.168.1.2',100)

v.set_velocity(left_motor,right_motor)
v.set_lights(lights)
v.set_vertical(vertical_motor)
print v.send_command()
v.toggle_camera(True)
v.send_command()


