# Message that specifies the robot's health as known by the Robot Controller
#
# Author: Mark Desnoyer (markd@cmu.edu)
# Date: July 2010

# Voltage going to the robot in Volts. Note, as of initial deployment,
# this will probably be either 48V or 0V because we don't actually
# have a volt meter.
float32 voltage

# Do we have a comms link to the router
bool router_comms_ok

# Do we have a comms link to the robot
bool robot_comms_ok

# Are the various motors operational
bool left_motor_ok
bool right_motor_ok
bool vertical_motor_ok

# Are we getting valid readings from the depth sensor
bool depth_sensor_ok

# Are we getting valid readings from the heading sensor
bool heading_sensor_ok

# Error code reported by the robot
int32 robot_error_code
