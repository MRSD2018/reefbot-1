# A message that says to start logging some video
#
# Author: Mark Desnoyer (markd@cmu.edu)
# Date: Oct 2010

# The ip of the broadcast stream
std_msgs/String rtp_ip

# The port of the broadcast stream
std_msgs/String rtp_port

# How long to log the video in seconds. 0 means go until we die.
float64 duration