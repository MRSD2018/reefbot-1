# This message specifies the physical status of the robot as known by
# the RobotController. This will be used to display the status to the
# user.
#
# Author: Mark Desnoyer (markd@cmu.edu)
# Date: July 2010

Header header

# Speeds of all the thursters. Positive is forward or up
# TODO(mdesnoyer, furlong) define the units (RPM maybe?)
float32 left_speed
float32 right_speed
float32 vertical_speed

# Depth of the robot in meters according to the depth sensor
float32 depth

# Compass heading in degrees where 0 is N, 90 is E, 180 is S and 270 is W
float32 heading
float32 roll
float32 pitch

float32 internal_humidity
float32 water_temp

float32 spin_count

float32 internal_temp
float32 total_power
float32 voltage_drop
float32 tether_voltage
float32 bus_voltage
float32 bus_current
float32 comm_error_count

