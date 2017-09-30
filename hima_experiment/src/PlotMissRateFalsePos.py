#!/usr/bin/python
'''Plots the precision recall graph for a single run. It is generated using the scored detections and changing the detection threshold.
'''
usage='PlotMissRateFalsePosl.py [options]'

import roslib; roslib.load_manifest('hima_experiment')
import rospy
from pylab import *
import numpy as np
from optparse import OptionParser
import math
import re
import os.path
import rosbag

from DetectorMetrics import *

if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('--input', '-i', dest='input',
                    help='Input bag to use',
                    default=None)
  parser.add_option('--do_object_memory', action='store_true',
                    dest='do_object_memory', default=False,
                    help='Add an analysis that assumes objects stay where they were if they were not looked at')
  parser.add_option('--min_overlap', type='float',
                    help='Minimum fraction overlap for a target to be correct',
                    default=0.5)
  parser.add_option('--do_min_overlap', action='store_true',
                    dest='do_min_overlap', default=False,
                    help='Use the minimum fraction overlap for the metric')
  parser.add_option('--do_min_bbox', action='store_true',
                    dest='do_min_bbox', default=False,
                    help='Use the minimum bounding box metric')
  parser.add_option('--width_ratio', type='float', default=0.41,
                    help='Standard ratio of width to height for a person. Default value is from Dollar et. al. 2001')
  parser.add_option('--title', default=None,
                    help='Title for the graph')


  (options, args) = parser.parse_args()

  if (options.do_min_overlap):
    detectorMetric = BoundingBoxOverlap(-float('inf'),
                                        options.do_object_memory,
                                        options.width_ratio,
                                        options.min_overlap)
  elif (options.do_min_bbox):
    detectorMetric = MinimumBoundingBox(-float('inf'),
                                        options.do_object_memory,
                                        options.width_ratio)

  # The input can be a comma separated list, so look for it
  inputs = options.input.split(',')

  for curInput in inputs:
    detectorMetricCopy = copy.deepcopy(detectorMetric)
    
    bag = rosbag.Bag(curInput)
    try:
      for topic, msg, t in bag.read_messages(topics=['results']):
        detectorMetricCopy.ProcessFrame(msg)
    finally:
      bag.close()

    # Get the set of possible scores
    scores = set([x[1] for x in detectorMetricCopy.detections])
    if len(scores) > 500:
      scores = np.linspace(min(scores), max(scores), 500)
    else:
      scores = sorted(scores)

    # Calculate the false positives and miss rates
    missRate = []
    falsePos = []
    for score in scores:
      detectorMetricCopy.scoreThresh = score

      missRate.append(detectorMetricCopy.GetMissRate())
      falsePos.append(detectorMetricCopy.GetFalsePosPerFrame())

    loglog(falsePos, missRate)

  
  xlabel('False Positives / Frame')
  ylabel('Miss Rate')
  legend([os.path.basename(x) for x in inputs], loc='lower left')

  if options.title is not None:
    title(options.title)

  show()
  

  
