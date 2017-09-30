#!/usr/bin/python
'''Script that creates a graph for the classification accuracy vs. data processed.
'''
usage='PlotDataProcessedGraph.py'

import roslib; roslib.load_manifest('hima_experiment')
import rospy
from pylab import *
import numpy as np
from optparse import OptionParser
import math
import re
import os.path
import cv
import rosbag
import copy
from DetectorMetrics import *

def GetBagFiles(experimentDir, bagRegexp):
  '''Looks in the experiment directory to find all the files with results from the experiment.

  Returns: A list of filenames for bags to open
  '''
  retVal = []
  for filename in os.listdir(experimentDir):
    if bagRegexp.match(filename):
      retVal.append(filename)

  return retVal  

def ProcessBag(bagFile, detectorMetric):
  '''Parses the bag for the information about the experiment

  Returns a tuple of (dataProcessed, precision, recall, timeInDetector,
  timeInFilter, objectPositionError) for the bag.
  '''

  # Copy the detector metric because it can have state
  detectorMetricCopy = copy.deepcopy(detectorMetric)

  # First determine the framesize by reading the parameters
  fracFramesize = 1.0
  fracResample = 1.0
  doVuFilter = True
  bag = rosbag.Bag(bagFile)
  try:
    for topic, msg, t in bag.read_messages(topics=['parameters']):
      if msg.name.data == 'fracFramesize':
        fracFramesize = float(msg.value.data)
      if msg.name.data == 'fracResample':
        fracResample = float(msg.value.data)
      if (msg.name.data == 'doVuFilter' and msg.value.data == 'False'):
        doVuFilter = False
  finally:
    bag.close()

  # Go through all the results
  imgSize = None
  bag = rosbag.Bag(bagFile)
  try:
    for topic, msg, t in bag.read_messages(topics=['results']):
      if imgSize is None:
        curImage = cv.LoadImage(msg.image)
        imgSize = (curImage.width, curImage.height)
        
      detectorMetricCopy.ProcessFrame(msg)
     
  finally:
    bag.close()

  # Go through the measured timing for the visual utility filter
  bag = rosbag.Bag(bagFile)
  detectorTimes = []
  try:
    for topic, msg, t in bag.read_messages(topics=['person_detector/processing_time']):
      if topic == 'person_detector/processing_time':
        detectorTimes.append(msg.data.to_sec())
  finally:
    bag.close()

  filterTime = 0.0
  if doVuFilter:
    bag = rosbag.Bag(bagFile)
    filterTimes = []
    try:
      for topic, msg, t in bag.read_messages(topics=['vufilter/processing_time']):
        filterTimes.append(msg.data.to_sec())
    finally:
      bag.close()
    filterTime = sum(filterTimes) / len(filterTimes)

  dataProcessed = detectorMetricCopy.GetDataProcessed()
  if dataProcessed < 1:
    dataProcessed = imgSize[0]*imgSize[1]*fracResample*fracResample

  return (dataProcessed,
          detectorMetricCopy.GetPrecision(),
          detectorMetricCopy.GetRecall(),
          sum(detectorTimes) / len(detectorTimes),
          filterTime,
          detectorMetricCopy.GetObjectPositionError(),
          detectorMetricCopy.GetTargetMissRate(),
          detectorMetricCopy.GetTargetMissWindow())
    
if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('--experiment_dir', dest='experiment_dir',
                    help='Directory containing the results of the experiment',
                    default='.')
  parser.add_option('--bag_regexp',
                    help='Regular expression to match the name of files to include',
                    default='.*frame.*\.bag')
  parser.add_option('--control_bag',
                    help='Bag with the control values for calculating timing',
                    default=None)
  parser.add_option('--min_overlap', type='float',
                    help='Minimum fraction overlap for a target to be correct',
                    default=0.5)
  parser.add_option('--do_min_overlap', action='store_true',
                    dest='do_min_overlap', default=False,
                    help='Use the minimum fraction overlap for the metric')
  parser.add_option('--do_min_bbox', action='store_true',
                    dest='do_min_bbox', default=False,
                    help='Use the minimum bounding box metric')

  parser.add_option('--score_thresh', type='float', default=0,
                    help='Threshold for the detection score to include result')
  parser.add_option('--do_object_memory', action='store_true',
                    dest='do_object_memory', default=False,
                    help='Add an analysis that assumes objects stay where they were if they were not looked at')
  parser.add_option('--width_ratio', type='float', default=0.41,
                    help='Standard ratio of width to height for a person. Default value is from Dollar et. al. 2001')

  


  (options, args) = parser.parse_args()

  if (options.do_min_overlap):
    detectorMetric = BoundingBoxOverlap(options.score_thresh,
                                        options.do_object_memory,
                                        options.width_ratio,
                                        options.min_overlap)
  elif (options.do_min_bbox):
    detectorMetric = MinimumBoundingBox(options.score_thresh,
                                        options.do_object_memory,
                                        options.width_ratio)

  bagRegexp = re.compile(options.bag_regexp)

  bagFiles = GetBagFiles(options.experiment_dir, bagRegexp)

  dataProcessed = []
  precision = []
  recall = []
  detectorTimes = []
  filterTimes = []
  posErrors = []
  targetMissRates = []
  targetMissWindows = []
  for bagFile in bagFiles:
    try:
      (data, curPrecision, curRecall, detectorTime, filterTime, posError,
       targetMissRate, targetMissWindow) = \
             ProcessBag(os.path.join(options.experiment_dir, bagFile),
                        detectorMetric)
    except rosbag.ROSBagFormatException as e:
      rospy.logerr('Error reading bag %s: %s' % (bagFile, str(e)))
      continue
    dataProcessed.append(data)
    precision.append(curPrecision)
    recall.append(curRecall)
    detectorTimes.append(detectorTime)
    filterTimes.append(filterTime)
    posErrors.append(posError)
    targetMissRates.append(targetMissRate)
    targetMissWindows.append(targetMissWindow)
  dataProcessed = np.array(dataProcessed)
  precision = np.array(precision)
  recall = np.array(recall)
  detectorTimes = np.array(detectorTimes)
  filterTimes = np.array(filterTimes)
  posErrors = np.array(posErrors)
  targetMissRates = np.array(targetMissRates)
  targetMissWindows = np.array(targetMissWindows)
  sortI = np.argsort(dataProcessed)

  # Open up the control bag
  controlResults = ProcessBag(os.path.join(options.experiment_dir,
                                           options.control_bag),
                              detectorMetric)

  dataProcessed = np.concatenate((dataProcessed[sortI], [controlResults[0]]))
  precision = np.concatenate((precision[sortI], [controlResults[1]]))
  recall = np.concatenate((recall[sortI], [controlResults[2]]))
  detectorTimes = np.concatenate((detectorTimes[sortI], [controlResults[3]]))
  filterTimes = np.concatenate((filterTimes[sortI], [controlResults[4]]))
  posErrors = np.concatenate((posErrors[sortI], [controlResults[5]]))
  targetMissRates = np.concatenate((targetMissRates[sortI],
                                    [controlResults[6]]))
  targetMissWindows = np.concatenate((targetMissWindows[sortI],
                                      [controlResults[7]]))
                              

  # make a precision and recall vs. data processed graph
  figure(1)
  plot(dataProcessed, precision, label='Precision')
  plot(dataProcessed, recall, label='Recall')
  xlabel('Data Processed (pixels)')
  title(detectorMetric.GetMetricName())
  legend(loc='lower right')

  # make a processing time vs. data processed
  figure(2)
  plot(dataProcessed, detectorTimes, label='Detector')
  plot(dataProcessed, filterTimes, label='Visual Utility Filter')
  plot(dataProcessed, filterTimes+detectorTimes,
       label='Total')
  ylabel('Time (s)')
  xlabel('Data Processed (pixels)')
  title(detectorMetric.GetMetricName())
  legend(loc='upper left')

  # Make a Object Position Error graph vs. data processed
  figure(4)
  plot(dataProcessed, posErrors)
  xlabel('Data Processed (pixels)')
  ylabel('Mean Object Position Error (pixels)')
  title(detectorMetric.GetMetricName())

  # Make a precision and recall vs. speedup
  figure(3)
  speedup = controlResults[3] / (filterTimes+detectorTimes)
  plot(speedup, precision, label='Precision')
  plot(speedup, recall, label='Recall')
  xlabel('Speedup')
  title(detectorMetric.GetMetricName())
  legend()

  # Make a Target Miss Rate vs. speedup
  figure(5)
  plot(speedup, targetMissRates, label='Target Miss Rate')
  xlabel('Speedup')
  ylabel('Target Miss Rate')
  title(detectorMetric.GetMetricName())

  # Make a Target Miss Window vs. speedup
  figure(6)
  plot(speedup, targetMissWindows, label='Target Miss Window')
  xlabel('Speedup')
  ylabel('Time (s)')
  title(detectorMetric.GetMetricName())
  

  
  show()
