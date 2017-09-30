# Message that species a region in the image to look at to identify
# the species.
#
# Author: Mark Desnoyer (markd@cmu.edu)
# Date: June 2010

# Bounding box in the image that specifies the region
sensor_msgs/RegionOfInterest bounding_box

# Binary mask image specifying where the species is in the bounding
# box. A pixel value of zero means to ignore the pixel. If this image
# is empty, then the entire box should be used, but if it's not empty,
# it must be the same size as bbox or else an error will be generated.
sensor_msgs/Image mask