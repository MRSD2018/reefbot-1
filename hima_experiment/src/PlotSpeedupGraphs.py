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

def GetBagFiles(experimentDirs, bagRegexp, controlRegexp, setRegexp):
  '''Looks in the experiment directory to find all the files with results from the experiment.

  Returns: A list of (filename, setId, isControl) tuples
  '''
  retVal = []
  for d in experimentDirs.strip().split(','):
    for filename in os.listdir(d):
      if bagRegexp.match(filename):
        fullFilename = os.path.join(d, filename)
        setId = setRegexp.search(filename).groups()[0]
        retVal.append((fullFilename, setId, False))

      if controlRegexp.match(filename):
        setId = setRegexp.search(filename).groups()[0]
        retVal.append((os.path.join(d, filename), setId, True))

  return retVal  

def ProcessBag(bagFile, detectorMetric, dropFullFrames=False):
  '''Parses the bag for the information about the experiment

  Returns a tuple of (dataProcessed, precision, recall, timeInDetector,
  timeInFilter) for the bag.
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
        
      detectorMetricCopy.ProcessFrame(msg, dropFullFrames, imgSize)
     
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
          detectorMetricCopy.GetTargetMissRate(),
          detectorMetricCopy.GetTargetMissWindow())
    
if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('--experiment_dirs', 
                    help='Comma separated list of directories containing the results of the experiment',
                    default='.')
  parser.add_option('--bag_regexp',
                    help='Regular expression to match the name of files to include',
                    default='.*frame.*\.bag')
  parser.add_option('--set_regexp', default='set([0-9]+)',
                    help='Regexp to extract the set number from a bag filename')
  parser.add_option('--control_regexp',
                    help='Matching regexp for matching the control bag',
                    default='.*control\.bag')
  parser.add_option('--min_overlap', type='float',
                    help='Minimum fraction overlap for a target to be correct',
                    default=0.5)

  parser.add_option('--score_thresh', type='float', default=0,
                    help='Threshold for the detection score to include result')
  parser.add_option('--do_object_memory', action='store_true',
                    dest='do_object_memory', default=False,
                    help='Add an analysis that assumes objects stay where they were if they were not looked at')
  parser.add_option('--width_ratio', type='float', default=0.41,
                    help='Standard ratio of width to height for a person. Default value is from Dollar et. al. 2001')
  parser.add_option('--group_names', default=None,
                    help='Python code specifying the list of group names to use. Must be one per directory')

  


  (options, args) = parser.parse_args()

  detectorMetric = BoundingBoxOverlap(options.score_thresh,
                                      options.do_object_memory,
                                      options.width_ratio,
                                      options.min_overlap)

  bagRegexp = re.compile(options.bag_regexp)
  controlRegexp = re.compile(options.control_regexp)
  setRegexp = re.compile(options.set_regexp)

  bagFiles = GetBagFiles(options.experiment_dirs, bagRegexp, controlRegexp,
                         setRegexp)

  bagData = []
  for bagFile in bagFiles:
    try:
      (dataProcessed, curPrecision, curRecall, detectorTime, filterTime,
       targetMissRate, targetMissWindow) = \
       ProcessBag(bagFile[0],
                  detectorMetric,
                  False)
    except rosbag.ROSBagFormatException as e:
      rospy.logerr('Error reading bag %s: %s' % (bagFile, str(e)))
      continue
    bagData.append((
      bagFile[0], # filename
      bagFile[1], # setid
      bagFile[2], # iscontrol
      dataProcessed,
      curPrecision,
      curRecall,
      detectorTime,
      filterTime,
      targetMissRate,
      targetMissWindow))

  groupColors = ['b', 'g', 'r', 'k', 'm', 'y']
  lineStyles = ['-', '--', ':', '-.', '-', '-']
  curGroup = 0
  precisionLines = []
  recallLines = []
  scatterPoints = []
  missRateLines = []
  missWindowLines = []

  for curDir in options.experiment_dirs.strip().split(','):
    datasetBags = filter(lambda x: os.path.samefile(os.path.dirname(x[0]), curDir),
                         bagData)

    setList = set([x[1] for x in datasetBags])
    firstLine = True
    for setId in setList:
      setBags = filter(lambda x: x[1] == setId and not x[2], datasetBags)
      setBags = sorted(setBags, key=lambda x: x[6] + x[7])
      controlBag = filter(lambda x: x[1] == setId and x[2], datasetBags)[0]
      controlTime = float(controlBag[6] + controlBag[7])

      precision = np.array([x[4] for x in setBags]) / float(controlBag[4])
      recall = np.array([x[5] for x in setBags]) / float(controlBag[5])
      detectorTime = np.array([x[6] for x in setBags])
      filterTime = np.array([x[7] for x in setBags])
      dataProcessed = np.array([x[3] for x in setBags])
      speedup = np.array([controlTime / (x[6] + x[7]) for x in setBags])
      targetMissRate = np.array([x[8] for x in setBags])
      targetMissWindow = np.array([x[9] for x in setBags])

      print 'Set %s\nMiss Rate: %s\nMissWindow: %s' % (setId, targetMissRate,
                                                       targetMissWindow)

      # Draw the line for the precision graph
      figure(1)
      curLines = plot(speedup, precision, groupColors[curGroup], hold=True,
                      linewidth=3, ls=lineStyles[curGroup])
      if firstLine:
        precisionLines.extend(curLines)

      # Draw the line on the recall graph
      figure(2)
      curLines = plot(speedup, recall, groupColors[curGroup], hold=True,
                      linewidth=3, ls=lineStyles[curGroup])
      if firstLine:
        recallLines.extend(curLines)

      # Draw the points on the data processed graph
      figure(3)
      scatCol = plot(dataProcessed, filterTime, c='b', marker='o',
                     hold=True, label='Visual Utility Filtering',
                     linewidth=3)
      if len(scatterPoints) < 4: scatterPoints.append(scatCol)
        
      scatCol = plot(dataProcessed, detectorTime, c='g', marker='x',
                     hold=True, label='Detector',
                     linewidth=3)
      if len(scatterPoints) < 4: scatterPoints.append(scatCol)
      
      scatCol = plot(dataProcessed, filterTime+detectorTime, c='r',
                     marker='d', hold=True, label='Total',
                     linewidth=3)
      if len(scatterPoints) < 4: scatterPoints.append(scatCol)
      
      scatCol = scatter(controlBag[3], controlTime, c='k', marker='^',
                        hold=True, label='Full Frame Detector', linewidth=3)
      if len(scatterPoints) < 4: scatterPoints.append(scatCol)

      # Draw the line on the target miss rate graph
      figure(4)
      curLines = plot(speedup, targetMissRate, groupColors[curGroup], hold=True,
                      linewidth=3, ls=lineStyles[curGroup])
      if firstLine:
        missRateLines.extend(curLines)
      scatter(1.0, controlBag[8], c=groupColors[curGroup], marker='d')

      # Draw the line on the target miss rate graph
      figure(5)
      curLines = plot(speedup, targetMissWindow, groupColors[curGroup], hold=True,
                      linewidth=3, ls=lineStyles[curGroup])
      if firstLine:
        missWindowLines.extend(curLines)
      scatter(1.0, controlBag[9], c=groupColors[curGroup], marker='d')

      firstLine = False

    curGroup += 1

  figure(1)
  xlabel('Speedup')
  ylabel('Fraction of Precision When Processing Full Frame')
  if options.group_names is not None:
    legend(precisionLines, eval(options.group_names))

  figure(2)
  xlabel('Speedup')
  ylabel('Fraction of Recall When Processing Full Frame')
  if options.group_names is not None:
    legend(recallLines, eval(options.group_names))

  figure(3)
  xlabel('Number of Pixels Processed by the Detector')
  ylabel('Runtime (s)')
  legend(scatterPoints, ('Visual Utility Filter', 'Detector', 'Total', 'Control'), loc='upper left')

  figure(4)
  xlabel('Speedup')
  ylabel('Target Miss Rate')
  if options.group_names is not None:
    legend(missRateLines, eval(options.group_names))

  figure(5)
  xlabel('Speedup')
  ylabel('Target Miss Window (s)')
  if options.group_names is not None:
    legend(missWindowLines, eval(options.group_names))
  
  show()
