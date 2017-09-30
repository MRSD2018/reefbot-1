#!/usr/bin/python
'''Creates a video where each frame is annotated with color coded
frames based on the scores in a jet color scheme.
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
import colorsys
from DetectionUtils import *

def FixOffsets(bbox):
  # On some of the datasets, the bounding box can have negative
  # offsets which get encoded as 2^32-val. So fix that.

  retval = bbox
  if retval.x_offset > (1<<31):
    retval.width += retval.x_offset - (1<<32)
    retval.x_offset = 0
  if retval.y_offset > (1<<31):
    retval.height += retval.y_offset - (1<<32)
    retval.y_offset = 0

  return retval

def NormalizeParamName(paramName):
  '''Converts parameter name from camelcase to underscores.'''
  s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', paramName)
  return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

def ToNumIfPossible(v):
  '''Converts a value to a number if it is possible.'''
  try:
    return float(v)
  except ValueError: pass

  return v

def DrawRectangles(image, msg, score_thresh, color_ground_truth,
                   do_nms):

  # Get the list of boxes to draw
  validIdx = [i for i in range(len(msg.scores)) if
              msg.scores[i] > score_thresh]
  regionScores = [((msg.regions[i].x_offset,
                    msg.regions[i].y_offset,
                    msg.regions[i].height,
                    msg.regions[i].width),
                   msg.scores[i]) for i in validIdx]
  overlaps = [msg.overlaps[i] for i in validIdx]
  if do_nms:
    regionScores = ApplyNMS(regionScores, float('nan'))
    validIdx = [j for j in range(len(validIdx)) if
                isfinite(regionScores[j][1])]
    boxes = [regionScores[j][0] for j in validIdx]
    scores = [regionScores[j][1] for j in validIdx]
    overlaps = [overlaps[j] for j in validIdx]
  else:
    boxes = [x[0] for x in regionScores]
    scores = [x[1] for x in regionScores]

  if len(scores) == 0:
    return image

  maxScore = max(scores)
  minScore = min(scores)
  for i in range(len(scores)):
    bbox = boxes[i]
    
    # Pick a color based on the score so that red is the highest and
    # blue lowest
    if color_ground_truth:
      #color = colorsys.hsv_to_rgb(2.0/3 - 2.0*overlaps[i]/3,
      #                            0.8, 1.0)
      if overlaps[i] > 0.5:
        color = (1.0, 0.0, 0.0)
      else:
        color = (0.0, 0.0, 1.0)
    else:
      if minScore == maxScore:
        color = (1.0, 0, 0)
      else:
        color = colorsys.hsv_to_rgb(2.0/3 - 2.0*(scores[i]-minScore)/
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
  parser.add_option('--output', '-o', dest='output',
                    help='Filename of the output video',
                    default=None)
  parser.add_option('--score_thresh', type='float', default=0,
                    help='Threshold for the detection score to include result')
  parser.add_option('--color_ground_truth', action='store_true',
                    default=False, dest='color_ground_truth',
                    help='Should the color of the box come from the overlap fraction with the ground truth ?')
  parser.add_option('--do_nms', action='store_true', default=False,
                    help='Do non maximal suppression of the HOG scores?')


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

    maxScore = max((maxScore, max(msg.scores)))

    # Open the current image
    image = cv2.imread(msg.image)

    # Show the original image and the markup
    outputFrame = np.zeros_like(image)
    smallImage = sp.misc.imresize(image, 0.5)
    outputFrame[0:smallImage.shape[0], 0:smallImage.shape[1], :] = smallImage


    # The rectangles on the frame
    markedImage = DrawRectangles(np.copy(image), msg, options.score_thresh,
                                 options.color_ground_truth,
                                 options.do_nms)
    markedImage = sp.misc.imresize(markedImage, 0.5)
    outputFrame[0:smallImage.shape[0], smallImage.shape[1]:, :] = markedImage
      
    videoOutput.write(outputFrame)

    nFrames = nFrames + 1
    if nFrames % 50 == 0:
      rospy.loginfo('Processed %i frames' % nFrames)

  del videoOutput
  rospy.loginfo('The maximum score was: %f' % maxScore)
  
                   

  
