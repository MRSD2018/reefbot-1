#!/usr/bin/python
'''Plots the precision recall graph for a single run. It is generated using the scored detections and changing the detection threshold.
'''
usage='PlotPrecisionRecall.py'

import roslib; roslib.load_manifest('hima_experiment')
import rospy
from pylab import *
import numpy as np
from optparse import OptionParser
import math
import re
import os.path
import rosbag
import cv
import scipy.integrate

from DetectorMetrics import *

def GetBagFiles(input, doGroups, groupNames, setRegexp):
  '''Get the list of bag files to process.

  Returns list of (bagFile, groupName, setId) tuples
  '''
  retval = []
  curGroup = 0
  if doGroups:
    for groupFile in input.split(','):
      for line in open(groupFile):
        groupName = os.path.basename(groupFile)
        if groupNames is not None:
          groupName = groupNames[curGroup]
        setId = setRegexp.search(line).groups()[0]
        retval.append((line.strip(), groupName, setId))

      curGroup += 1
  else:
    for filename in input.split(','):
      groupName = os.path.basename(filename)
      if groupNames is not None:
        groupName = groupNames[curGroup]
      setId = setRegexp.search(filename).groups()[0]
      retval.append((filename.strip(), groupName, setId))
    curGroup += 1

  return retval

def ProcessBag(bagFiles, detectorMetric, dropFullFrames=False):
  '''Calculates the precision and recall for a bag.

  returns tuple of (precision, recall) where is is a numpy array
  '''
  detectorMetricCopy = copy.deepcopy(detectorMetric)
  for bagFile in bagFiles:
    
    imgSize = None
    bag = rosbag.Bag(bagFile)
    try:
      for topic, msg, t in bag.read_messages(topics=['results']):
        if imgSize is None:
          curImage = cv.LoadImage(msg.image)
          imgSize = (curImage.width, curImage.height)
        detectorMetricCopy.ProcessFrame(msg, dropFullFrames, imgSize)
    finally:
      bag.close()

  return CalculatePR(detectorMetricCopy)

def CalculatePR(detectorMetric):
  # Get the set of possible scores
  scores = set([x[1] for x in detectorMetric.detections])
  if len(scores) > 400:
    scores = np.linspace(min(scores), max(scores), 400)
  else:
    scores = sorted(scores)

  # Calculate the precision and recall for all the possible scores
  recall = []
  precision = []
  for score in scores:
    detectorMetric.scoreThresh = score
    
    recall.append(detectorMetric.GetRecall())
    precision.append(detectorMetric.GetPrecision())

  return (np.array(precision), np.array(recall))

if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('--input', '-i', dest='input',
                    help='Input bag to use',
                    default=None)
  parser.add_option('--control_group', default=None,
                    help='File specifying the control bags')                 
  parser.add_option('--do_object_memory', action='store_true',
                    dest='do_object_memory', default=False,
                    help='Add an analysis that assumes objects stay where they were if they were not looked at')
  parser.add_option('--min_overlap', type='float',
                    help='Minimum fraction overlap for a target to be correct',
                    default=0.5)
  parser.add_option('--width_ratio', type='float', default=0.41,
                    help='Standard ratio of width to height for a person. Default value is from Dollar et. al. 2001')
  parser.add_option('--title', default=None,
                    help='Title for the graph')
  parser.add_option('--do_groups', action='store_true', default=False,
                    help='Should the input be a list of files, each of which contains a list of bags to plot?')
  parser.add_option('--group_names', default=None,
                    help='Python code specifying the list of group names to use. Must be one per directory')
  parser.add_option('--set_regexp', default='set([0-9]+)',
                    help='Regexp to extract the set number from a bag filename')


  (options, args) = parser.parse_args()


  detectorMetric = BoundingBoxOverlap(-float('inf'),
                                      options.do_object_memory,
                                      options.width_ratio,
                                      options.min_overlap)
  setRegexp = re.compile(options.set_regexp)
  bagFiles = GetBagFiles(options.input, options.do_groups,
                         eval(options.group_names),
                         setRegexp)

  controlResults = {}
  if options.do_groups:
    controlBags = GetBagFiles(options.control_group, True, ('Control'),
                              setRegexp)
    #for bagFile, groupName, setId in controlBags:
    #  controlResults[setId] = ProcessBag(bagFile, detectorMetric, False)
    

  groupList = set([x[1] for x in bagFiles])
  groupColors = ['b-', 'g--', 'r:', 'k-.', 'm']
  curGroup = 0
  lines = []

  for group in groupList:
    inputs = [x for x in bagFiles if x[1] == group]
    firstLine = True
    precision, recall = ProcessBag([x[0] for x in inputs], detectorMetric,
                                   True)
    curLine = plot(recall, precision, groupColors[curGroup], linewidth=3)
    if firstLine:
      lines.append(curLine)
      firstLine = False

    # Calculate the average precision
    print 'Average precision for %s is: %f' % (group,
                                               scipy.integrate.trapz(
                                                 precision, recall))
        
   ## for curInput, groupName, setId in inputs:
##      precision, recall = ProcessBag(curInput, detectorMetric, True)

##      # Adjust precision and recall to be a fraction of the control
##      if len(controlResults) > 0:
##        controlP, controlR = controlResults[setId]
##        precision = precision / np.interp(recall, controlR, controlP, 1, 0)

##      # Plot the PR curve
##      curLine = plot(recall, precision, c=groupColors[curGroup])
##      if firstLine:
##        lines.append(curLine)
##        firstLine = False

##      # Calculate the average precision
##      precision = np.array(precision)
##      recall = np.array(recall)
##      precisionSum = 0.0
##      for start in arange(0,1,0.1):
##        validVals = precision[np.logical_and(recall > start,
##                                             recall <= start+0.1)]
##        if len(validVals) > 0:
##          precisionSum += np.mean(validVals)
##      print 'Average precision for %s is: %f' % (curInput, precisionSum/10)

    curGroup += 1

  
  xlabel('Recall')
  ylabel('Precision')
  legend(lines, groupList, loc='upper right')

  if options.title is not None:
    title(options.title)

  show()
  

  
