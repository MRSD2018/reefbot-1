# A message specifying the frame and the response of the person detector
Header header

# Filename of the image
string image

PersonLocation[] groundTruth

cascade_parts_detector/DetectionArray detectorResponse

# The regions of interest that was processed by the person detector
sensor_msgs/RegionOfInterest[] roi