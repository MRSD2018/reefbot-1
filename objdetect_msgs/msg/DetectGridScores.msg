# Specifies socres on a detection grid that runs (x,y,w,h). If the aspect ratio is fixed, this will change to (x,y,s)

Header header

# The (w,h,x,y) grid that has a response
Grid grid

# A grid of scores across the space that are based on an evaluation
# for each box.
sensor_msgs/MatND scores

# An optional binary mask that is 4 dimensional (w,h,x,y) and
# specifies which entries have valid values
sensor_msgs/MatND mask

# The processing time to calculate the detection
std_msgs/Duration processing_time