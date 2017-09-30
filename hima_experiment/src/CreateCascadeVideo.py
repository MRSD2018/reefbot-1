#!/usr/bin/python
'''Creates a video where each frame is annotated with boxes where blue
are those that pass the cascade and red are those that pass the high
level detector.
'''
usage='CreateVUEstimationVideo.py'

import roslib; roslib.load_manifest('hima_experiment')
import rospy
from pylab import *
import numpy as np
import scipy as sp
from optparse import OptionParser
import math
import re
import os.path
import cv2
import rosbag
from DetectionUtils import *

def DrawRectangles(image, msg, hogOnly=False):

  for i in range(len(msg.pass_vu_regions)):
    regionI = msg.pass_vu_regions[i]
    bbox = (msg.total_regions[regionI].x_offset,
            msg.total_regions[regionI].y_offset,
            msg.total_regions[regionI].height,
            msg.total_regions[regionI].width)

    # Pick the color based on if it passed the HOG threshold
    if msg.scores[i] > 0:
      color = [1.0, 0.0, 0.0]
    else:
      color = [0.0, 0.0, 1.0]
      if hogOnly:
        continue

    if msg.overlaps[regionI] > 0.5:
      color[1] = 1.0

    cv2.rectangle(image,
                  (bbox[0], bbox[1]),
                  (bbox[0] + bbox[3],
                   bbox[1] + bbox[2]),
                  cv2.cv.RGB(color[0]*255, color[1]*255, color[2]*255),
                  2) # thickness
  return image

if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('--input', '-i', dest='input',
                    help='Input bag to use',
                    default=None)
  parser.add_option('--output', '-o', dest='output',
                    help='Filename of the output video',
                    default=None)


  (options, args) = parser.parse_args()

  # Open the input bag to read
  bag = rosbag.Bag(options.input)

  # Figure out the framesize
  exampleMsg = bag.read_messages(topics=['results']).next()[1]
  exampleImage = cv2.imread(exampleMsg.image)
  frameSize = (exampleImage.shape[1], exampleImage.shape[0])

  # Open the video for output
  videoOutput = cv2.VideoWriter(options.output,
                                cv2.cv.CV_FOURCC('M', 'P', 'E', 'G'),
                                30.0, # fps needs to be 30 for MPEG
                                frameSize,
                                1) # is_color

  # Now go through the frames in the bag and make the video
  nFrames = 0
  lastMsg = None
  foundTargets = set()
  maxScore = -float('inf')
  for topic, msg, t in bag.read_messages(topics=['results']):
    if topic <> 'results':
      continue

    # Open the current image
    image = cv2.imread(msg.image)

    # Show the original image and the markup
    outputFrame = np.zeros_like(image)
    smallImage = sp.misc.imresize(image, 0.5)
    outputFrame[0:smallImage.shape[0], 0:smallImage.shape[1], :] = smallImage

    # Draw rectangles on the frame for what went past the cascade
    markedImage = DrawRectangles(np.copy(image), msg)
    markedImage = sp.misc.imresize(markedImage, 0.5)
    outputFrame[smallImage.shape[0]:, 0:smallImage.shape[1], :] = markedImage

    # Draw rectangles for what was triggered by the hog detector
    markedImage  = DrawRectangles(np.copy(image), msg, hogOnly=True)
    markedImage = sp.misc.imresize(markedImage, 0.5)
    outputFrame[0:smallImage.shape[0], smallImage.shape[1]:, :] = markedImage
      
    videoOutput.write(outputFrame)

    nFrames = nFrames + 1
    if nFrames % 50 == 0:
      rospy.loginfo('Processed %i frames' % nFrames)

    
