#!/usr/bin/python
'''Converts a directory of frames into a bag file.

The bag file will have the proper timestamps and topics for the left and right camera datastreams.
'''
usage='hima_images2bag.py [options]' 

import roslib; roslib.load_manifest('hima_experiment')
import rospy
import rosbag
from sensor_msgs.msg import Image
from optparse import OptionParser
import os
import os.path
import glob
import cv
import cv_bridge

def WriteImageToBag(bag, topic, imageFn, timestamp, frame_id):
  image = cv.LoadImage(imageFn)
  msg = cv_bridge.CvBridge().cv_to_imgmsg(image, 'bgr8')
  msg.header.stamp = rospy.Time.from_sec(timestamp)
  msg.header.frame_id = frame_id
  bag.write(topic, msg, msg.header.stamp)

if __name__ == '__main__':
  # All of the command line flags
  parser = OptionParser(usage=usage)

  parser.add_option('--left_image_dir',
                    help='Directory that contains all the left camera images',
                    default=None)
  parser.add_option('--right_image_dir',
                    help='Directory that contains all the right camera images',
                    default=None)
  parser.add_option('--timestamps',
                    help='File that contains the timestamps',
                    default=None)
  parser.add_option('-o', '--output', default='hima.bag',
                    help='Output bag file')

  (options, args) = parser.parse_args()

  # Get the frame id to timestamps
  timestamps = []
  for timestamp in open(options.timestamps):
    timestamps.append(float(timestamp.strip()))

  # Get the list of images
  rightImages = glob.glob(options.right_image_dir + '/*.bmp')
  rightImages = sorted(rightImages)
  leftImages = glob.glob(options.left_image_dir + '/*.bmp')
  leftImages = sorted(leftImages)

  # Open up the bag file to write to
  bag = rosbag.Bag(options.output, 'w')
  try:
    for i in range(len(timestamps)):
      WriteImageToBag(bag, 'left/image', leftImages[i], timestamps[i],
                      "/left_camera")
      WriteImageToBag(bag, 'right/image', rightImages[i], timestamps[i],
                      "/right_camera")

  finally:
    bag.close()
