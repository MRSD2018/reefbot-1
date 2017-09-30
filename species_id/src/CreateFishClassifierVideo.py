#!/usr/bin/python
'''Creates a video where each frame has boxes around the n regions with the top confidence in containing a fish in red. Ground truth fish are in blue
'''
usage='CreateFishClassifierVideo.py'

import roslib; roslib.load_manifest('species_id')
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
import colorsys
import cv_bridge

def DrawGridRectangles(image, fishConfidence, notFishConfidence, msg,
                       nRegions):

  # Get the list of boxes where we think there's a fish
  validIdx = np.nonzero(fishConfidence > notFishConfidence)
  if len(validIdx[0]) == 0:
    return image

  # Get the nth highest score
  sortedConfidence = np.sort(fishConfidence[validIdx])
  scoreThresh = sortedConfidence[0]
  if len(validIdx[0]) > nRegions:
    scoreThresh = sortedConfidence[-nRegions]

  validIdx = np.nonzero(np.logical_and(fishConfidence >= scoreThresh,                                                 fishConfidence > notFishConfidence))
  boxes = np.array([
    msg.grid.minX + validIdx[2]*msg.grid.strideX,
    msg.grid.minY + validIdx[3]*msg.grid.strideY,
    msg.grid.minW*np.power(msg.grid.strideW,validIdx[0]),
    msg.grid.minH*np.power(msg.grid.strideH,validIdx[1])])


  # Draw the boxes
  for i in range(len(validIdx[0])):
    bbox = boxes[:,i]
    
    cv2.rectangle(image,
                  (bbox[0], bbox[1]),
                  (bbox[0] + bbox[3],
                   bbox[1] + bbox[2]),
                  cv2.cv.RGB(200, 0, 0),
                  2) # thickness
  return image

def DrawGroundTruthRectangles(image, msg):
  for region in msg.regions:
    cv2.rectangle(image,
                  (region.x_offset, region.y_offset),
                  (region.x_offset + region.width,
                   region.y_offset + region.height),
                  cv2.cv.RGB(0, 0, 200),
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
  parser.add_option('--nRegions', default=10, type='int',
                    help='Draw boxes around the top N regions')
  parser.add_option('--flip_colors', default=False, action='store_true',
                    help='Should the R and B channels be flipped?')

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
  matBridge = cv_bridge.NumpyBridge()
  for topic, msg, t in bag.read_messages(topics=['results']):
    if topic <> 'results':
      continue

    fishConfidence = matBridge.matmsg_to_numpy(msg.confidence)
    notFish = matBridge.matmsg_to_numpy(msg.not_fish_confidence)

    # Open the current image
    curImage = cv2.imread(msg.image)
    if options.flip_colors:
      tmp = np.copy(curImage[:,:,0])
      curImage[:,:,0] = curImage[:,:,2]
      curImage[:,:,2] = tmp

    # Show the original image in a subwindow
    outputFrame = np.zeros_like(curImage)
    smallImage = sp.misc.imresize(curImage, 0.5)
    outputFrame[0:smallImage.shape[0], 0:smallImage.shape[1], :] = smallImage
    

    # The rectangles on the frame
    curImage = DrawGridRectangles(curImage, fishConfidence, notFish, msg,
                                  options.nRegions)
    curImage = DrawGroundTruthRectangles(curImage, msg)
    markedImage = sp.misc.imresize(curImage, 0.5)
    outputFrame[smallImage.shape[0]:, 0:markedImage.shape[1], :] = markedImage

    videoOutput.write(outputFrame)

    nFrames = nFrames + 1
    if nFrames % 50 == 0:
      rospy.loginfo('Processed %i frames' % nFrames)

  del videoOutput
  
                   

  
