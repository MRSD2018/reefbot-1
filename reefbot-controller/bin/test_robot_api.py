from robot_api import *
import sys

left_motor = 0
right_motor = 0
lights = 0

if len(sys.argv) >= 2:
	left_motor = float(sys.argv[1])
if len(sys.argv) >= 3:
	right_motor = float(sys.argv[2])
if len(sys.argv) >= 4:
	lights = int(sys.argv[3])

v = VideoRayAPI()
v.connect('192.168.1.2',100)

#v.set_velocity(left_motor,right_motor)
#v.set_lights(lights)
#print v.send_command()
v.toggle_camera(True)
v.send_command()

