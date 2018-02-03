# This message specifies that a still image was captured by the system.
#
# If you're using OpenCV to handle images, look up the cv_bridge in 
# ROS to easily write/read this message.
#
# Author: Mark Desnoyer (markd@cmu.edu)
# Date: Sept 2010

Header header

# Optional Id to identify the image
uint64 image_id

# Full color image that was captured
sensor_msgs/Image image
