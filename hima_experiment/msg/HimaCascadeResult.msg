# A message specifying the visual utility scores for a bunch of frames in an image
Header header

# Filename of the image
string image

# Processing time for the entire image through the entire cascade with
# the hog detector.
std_msgs/Duration processing_time

# Processing time for only the cascade visual utility estimator
std_msgs/Duration vu_processing_time

# The regions used by the visual utility estimator
sensor_msgs/RegionOfInterest[] total_regions

# The maximum overlap between the region and a real object
float32[] overlaps

# Indicies of regions that passed the visual utility estimator and
# were input into the high level detector.
int32[] pass_vu_regions

# The scores for each region that was evaluated by the high level detector
float32[] scores
