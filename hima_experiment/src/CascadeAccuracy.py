#!/usr/bin/python
'''Processing to get stats for a number of cascades.

options valid_vutype and estimator_regex can be used to extract the
different cascade models used.
'''

import roslib; roslib.load_manifest('hima_experiment')
import rospy
import rosbag
import math
import numpy as np
import scipy.interpolate
import os
import os.path
import re
import HimaDataLoader
import PlottingUtils
import cPickle as pickle
import cDetectionUtils
from optparse import OptionParser

VERSION='1'

class BagParse:
  '''Parses the results across all of the valid datasets.'''
  
  def __init__(self, bagFiles, annotationFiles, frameSubsetRate=1,
               overlapThresh=0.5, imageIdRegex=None, frameSubsetOffset=1):
    self.params = {} # bagFile -> {name -> value}
    self.results = {} # (fileId,region) -> (score, overlap)
    self.fileMap = {} # filename -> fileId
    self.fileIdMap = {} # fileId -> filename
    self.peopleLoc = {} # (imageFileId,personId)->PersonLocation
    self.nRegions = 0
    self.nImages = 0
    self.processingTime = []
    self.vuProcessingTime = []

    imageIdRe = re.compile(imageIdRegex)
    curFileId = 0
    curAnnotationsFile = 0
    fnPeopleLoc = {} # (imageFile, personId)->PersonLocation
    for bagFile in bagFiles:
      rospy.loginfo('Reading bag %s' % (bagFile))
      curParams = {}
      filenames = []
      bag = rosbag.Bag(bagFile)
      try:
        for topic, msg, t in bag.read_messages():
          if topic.endswith('parameters'):
            # Store the parameter as strings
            curParams[msg.name.data] = msg.value.data

          if topic == 'results':
            # See if we should import this message
            imageId = int(imageIdRe.search(msg.image).groups()[0])
            if imageId % frameSubsetRate == frameSubsetOffset:

              # Parse the result message
              self.processingTime.append(msg.processing_time.data.to_sec())
              self.vuProcessingTime.append(msg.vu_processing_time.data.to_sec())
              filenames.append(msg.image)
              for i in range(len(msg.scores)):
                regionMsg = msg.total_regions[msg.pass_vu_regions[i]]
                curRegion = (regionMsg.x_offset, regionMsg.y_offset,
                             regionMsg.height, regionMsg.width)
                self.results[(curFileId, curRegion)] = (
                  msg.scores[i],
                  msg.overlaps[msg.pass_vu_regions[i]])

              self.fileMap[msg.image] = curFileId
              self.fileIdMap[curFileId] = msg.image
              self.nRegions += len(msg.total_regions)
              self.nImages += 1

            curFileId += 1

        fnPeopleLoc.update(HimaDataLoader.LoadPeopleLocations(
          annotationFiles[curAnnotationsFile],
          filenames))
        curAnnotationsFile += 1

      finally:
        rospy.loginfo('Finished reading bag %s' % (bagFile))
        bag.close()
        self.params[bagFile] = curParams

    # Convert the people location dictionary to one that uses file ids
    # instead of filenames
    for peopleKey, peopleLoc in fnPeopleLoc.iteritems():
      self.peopleLoc[(self.fileMap[peopleKey[0]], peopleKey[1])] = \
                                                  peopleLoc

class CascadeAccuracyStats:
  def __init__(self, options):
    self.VERSION = int(VERSION)
    self.options = options # To record all the settings
    self.accuracy = []
    self.precision = []
    self.recall = []
    self.errorRate = []
    self.nWindows = []
    self.meanProcessingTime = []
    self.stdProcessingTime = []
    self.stdErrorProcTime = []
    self.vuRecall = []
    self.vuPrecision = []
    self.meanVuProcessingTime = []
    self.stdVuProcessingTime = []
    self.stdVuErrorProcTime = []

    annotationsMap = PlottingUtils.ParseAnnotationFiles(
      options,
      annotationDir=options.root_results_dir)
    bags, datasets, cascadeModels = PlottingUtils.FindVUBags(options,
                                                             useRoot=True)

    for model in cascadeModels:
      bagParse = BagParse([bags[(model, x)] for x in datasets],
                          [annotationsMap[x] for x in datasets],
                          options.frame_subset_rate,
                          options.min_overlap,
                          imageIdRegex=options.imageid_regex,
                          frameSubsetOffset=1)

      rospy.loginfo('Calculating stats for model: %s' % model)
      self.AppendStatsFromParse(bagParse, options.min_overlap,
                                options.hog_thresh)

  def AppendStatsFromParse(self, bagParse, minOverlap, hogThresh):

    # Grab the list of (fileId, box) groups
    regions = sorted(bagParse.results.keys())

    # Get an array of the HOG scores for all entries that passed the vu filter
    scores = np.array([bagParse.results[key][0] for key in regions])

    # Array of the overlaps with ground truth
    overlaps = np.array([bagParse.results[key][1] for key in regions])

    passHog = scores >= hogThresh
    groundTruth = overlaps > minOverlap
    hogPeoplePos = np.logical_and(groundTruth, passHog)

    # Calculate the precisions
    if len(regions) == 0:
      self.vuPrecision.append(float('nan'))
    else:
      self.vuPrecision.append(float(np.sum(groundTruth)) / len(regions))
    if np.sum(passHog) == 0:
      self.precision.append(float('nan'))
    else:
      self.precision.append(float(np.sum(hogPeoplePos)) / np.sum(passHog))

    # The recall of regions that pass the vu filter
    vuPersonOverlap = np.array(
      [np.any([personLoc.CalculateOverlapTuple(regionKey[1]) > minOverlap
               for regionKey in regions if
               regionKey[0] == peopleKey[0]])
       for peopleKey, personLoc in bagParse.peopleLoc.iteritems()])
    self.vuRecall.append(float(np.sum(vuPersonOverlap))/len(vuPersonOverlap))

    # TODO(mdesnoyer) Calculate the visual utility risk.  Cannot be
    # calculated at the moment because we didn't keep track of the
    # regions that would pass the high level hog detector and
    # correspond to a person.
    self.accuracy = None

    # Error rate
    self.errorRate = None # Not useful and can't calculate

    # Number of windows
    self.nWindows.append(len(scores) / bagParse.nImages)

    # Stats of the processing time
    self.meanProcessingTime.append(np.mean(bagParse.processingTime))
    stdProcTime = np.std(bagParse.processingTime)
    self.stdProcessingTime.append(stdProcTime)
    self.stdErrorProcTime.append(stdProcTime /
                                 math.sqrt(len(bagParse.processingTime)))

    self.meanVuProcessingTime.append(np.mean(bagParse.vuProcessingTime))
    stdProcTime = np.std(bagParse.vuProcessingTime)
    self.stdVuProcessingTime.append(stdProcTime)
    self.stdVuErrorProcTime.append(stdProcTime /
                                 math.sqrt(len(bagParse.vuProcessingTime)))

    # Recall, which is more complicated because we're looking at the
    # recall of people, not windows.
    setRegions = set(regions)
    peopleScores = np.array(
      [np.any([personLoc.CalculateOverlapTuple(regionKey[1]) > minOverlap
               for regionKey in regions if
               regionKey[0] == peopleKey[0] and
               bagParse.results[regionKey][0] > hogThresh])
       for peopleKey, personLoc in bagParse.peopleLoc.iteritems()])
    self.recall.append(float(np.sum(peopleScores)) / len(peopleScores))

if __name__ == '__main__':
  parser = OptionParser()

  #parser.add_option('--bags', help='Glob to specify the bags to process')
  parser.add_option('--imageid_regex', default='([0-9]+)\.((png)|(jpg)|(bmp))',
                    help='Regex to extract the image id from a filename')
  parser.add_option('--min_overlap', type='float',
                    help='Minimum fraction overlap for a target to be correct',
                    default=0.5)
  parser.add_option('--hog_thresh', type='float',
                    help='Minimum HOG value required to label a hit',
                    default=0.0)
  parser.add_option('--frame_subset_rate', type='int', default=1,
                    help='In order to run quicker, you can only run the detector on a subset of the frames. This specifies how often to run the detector. So, for example if it is 5, it will run the detector once every 5 frames.')
  parser.add_option('--output_data', '-o', default='CascadeAccuracy.stats',
                    help='Filename where the stats will be pickled to')

  parser = PlottingUtils.AddCommandLineOptions(parser)

  (options, args) = parser.parse_args()

  stats = CascadeAccuracyStats(options)
  rospy.loginfo('Saving calculated statistics to: %s' % options.output_data)
  dirName = os.path.dirname(options.output_data)
  if not os.path.exists(dirName) and dirName <> '':
    os.makedirs(dirName)
  outputFile = open(options.output_data, 'wb')
  try:
    pickle.dump(stats, outputFile)
  finally:
    outputFile.close()
