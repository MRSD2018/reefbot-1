#!/usr/bin/python
'''Creates a video where each frame is annotated with color coded
frames based on the scores in a jet color scheme.
'''
usage='CreateVUEstimationVideo.py'

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

def DrawRectangles(image, scores, msg, score_thresh):

  # Get the list of boxes to draw
  validIdx = np.nonzero(scores > score_thresh)
  if len(validIdx[0]) == 0:
    return image

  validScores = scores[validIdx]
  boxes = np.array([
    msg.scores.grid.minX + validIdx[2]*msg.scores.grid.strideX,
    msg.scores.grid.minY + validIdx[3]*msg.scores.grid.strideY,
    msg.scores.grid.minW*np.power(msg.scores.grid.strideW,validIdx[0]),
    msg.scores.grid.minH*np.power(msg.scores.grid.strideH,validIdx[1])])


  # Draw the boxes
  maxScore = max(validScores)
  minScore = min(validScores)
  for i in range(len(validScores)):
    bbox = boxes[:,i]
    
    # Pick a color based on the score so that red is the highest and
    # blue lowest
    if minScore == maxScore:
      color = (1.0, 0, 0)
    else:
      color = colorsys.hsv_to_rgb(2.0/3 - 2.0*(validScores[i]-minScore)/
                                  (maxScore-minScore)/3,
                                  0.8, 1.0)
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
  parser.add_option('--video_dir', help='Directory with the videos',
                    default='/data/mdesnoye/fish/tank_videos/20110102')
  parser.add_option('--output', '-o', dest='output',
                    help='Filename of the output video',
                    default=None)
  parser.add_option('--score_thresh', type='float', default=0,
                    help='Threshold for the detection score to include result')
  parser.add_option('--parsing_regex',
                    default='((([0-9][0-9]-){3})([0-9]+))\.',
                    help='Regex to extract the frame id and video id from filenames')
  


  (options, args) = parser.parse_args()

  parsingRe = re.compile(options.parsing_regex)

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

    curScores = matBridge.matmsg_to_numpy(msg.scores.scores)

    maxScore = max((maxScore, np.amax(curScores)))

    # Open the current image
    curImage = cv2.imread(msg.image)

    # Show the original image and the markup
    outputFrame = np.zeros_like(curImage)
    smallImage = sp.misc.imresize(curImage, 0.5)
    outputFrame[0:smallImage.shape[0], 0:smallImage.shape[1], :] = smallImage
    

    # The rectangles on the frame
    curImage = DrawRectangles(curImage, curScores, msg, options.score_thresh)
    markedImage = sp.misc.imresize(curImage, 0.5)
    outputFrame[smallImage.shape[0]:, 0:markedImage.shape[1], :] = markedImage

    videoOutput.write(outputFrame)

    nFrames = nFrames + 1
    if nFrames % 50 == 0:
      rospy.loginfo('Processed %i frames' % nFrames)

  del videoOutput
  rospy.loginfo('The maximum score was: %f' % maxScore)
  
                   

  
