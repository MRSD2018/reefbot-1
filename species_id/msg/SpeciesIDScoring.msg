# Filename of the image
string image

# List of the ground truth regions for where the fish are
sensor_msgs/RegionOfInterest[] regions

# Array of labels, one for each region that specify the ground truth species in each region from regions
uint32[] labels

# The grid defined for the classifier response
objdetect_msgs/Grid grid

# The top hit, the confidence and id label
sensor_msgs/MatND confidence
sensor_msgs/MatND top_label

# The confidence that it's not a fish
sensor_msgs/MatND not_fish_confidence

# The processing time of the request
std_msgs/Duration processing_time

