# To detect an object on a w,h,x,y grid. It is more compressed than listing all of the boxes we care about

Header header

# The image to find objects in
sensor_msgs/Image image

# The (w,h,x,y) grid to search on
Grid grid

# An optional binary mask that is 4 dimensional (w,h,x,y) and
# specifies which entries we actually want to search in
sensor_msgs/MatND mask



