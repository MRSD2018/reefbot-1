# This message specifies a new image and a label to add to the species id
# dictionary
#
# If you're using OpenCV to handle images, look up the cv_bridge in 
# ROS to easily write this message.
#
# Author: Mark Desnoyer (markd@cmu.edu)
# Date: Nov 2010

# Full color image that contains an instance of the species
sensor_msgs/Image image

# Regions in the image to look for individuals that we have labels for
ImageRegion[] regions

# Array of labels, one for each region that specify the species in that region
uint32[] labels

# Optional filename of the image
string filename

# Optional array of blob ids, one for each region specified
uint32[] blob_ids