# This message specifies a request to identify the species in a
# picture.  Only one species will be identified (the one that takes up
# most of the frame). For best results, use the mask to identify the
# exact pixels that the individual occupies.
#
# If you're using OpenCV to handle images, look up the cv_bridge in 
# ROS to easily write this message.
#
# Author: Mark Desnoyer (markd@cmu.edu)
# Date: June 2010

Header header

# Optional Id to identify the image
uint64 image_id

# Full color image that contains an instance of the species
sensor_msgs/Image image

# Regions in the image to look for individuals whose species need to
# be identified.
ImageRegion[] regions
