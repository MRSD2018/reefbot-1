s #!/usr/bin/python
'''Does the analysis for how different visual utility estimators impact the overall system performance.'''
usage='RunVUClassifierAnalysis.py'

import roslib; roslib.load_manifest('species_id')
import rospy
from species_id import DetectionUtils
import VUAnalysisTools
import numpy as np
import scipy.interpolate
from optparse import OptionParser
import re
import os
import os.path
import cv2
import rosbag
import cv_bridge
from cv_blobs import Blob
import cPickle as pickle
import hashlib
import struct
import math
import glob
import gzip

VERSION='3'

def LogAndRaiseError(err):
  rospy.logerr(err)
  raise err

def FindInputBags(inputs, videoRegex):
  '''Returns a dictionary of videoId->filename for the bags.'''
  videoRe = re.compile(videoRegex)
  retval = {}
  for f in glob.glob(inputs):
    match = videoRe.search(f)
    if match:
      retval[match.groups()[0]] = f

  return retval

def img2blobFn(imgFn, blobDir):
  imgRe = re.compile(r'(.*)\.(jpg|jpeg|bmp|png)')
  base = imgRe.match(os.path.basename(imgFn)).groups()[0]
  return os.path.join(blobDir, '%s.blob' % base)

class ClassifierParse:
  def InitData(self):
    # For each candidate box, the scores and overlaps
    self.scores = [] # Confidence of the best fish, output from classifier
    self.labels = [] # Labels of the best fish, output from classifier
    self.notFishConf = [] # Confidence from the classifier that it's not a fish
    self.overlaps = [] # Maximum overlap with a ground truth
    self.overlapLabels = [] # The species label of the best overlap
  
    # --The following 2 items are in the same order--  
    self.blobList = [] # list of (fileId, blobId)
    self.fishLoc = [] # list of (x,y,h,w) tuples of the ground truth for the fish
    
    self.fileMap = {} # filename -> fileId
    self.fileIdMap = {} # fileId -> filename
    self.params = {} # bagFile-> {name->value}
    self.grid = None # Grid specifications
    self.gridShape = None # The shape of the grid
    self.processingTime = [] # list of all the processing times for each frame
    self.md5 = None # An identifier for this object
  
  def __init__(self, inputBag, md5=None, frameSubsetRate=1, blobDir=None,
               overlapThresh=0.5):
    self.InitData()
    self.md5 = md5
    ndBridge = cv_bridge.NumpyBridge()

    curFileId = 0
    rospy.loginfo('Reading bag ' + bagFn)
    curParams = {}

    curBag = rosbag.Bag(bagFn)
    try:
      for topic, msg, t in curBag.read_messages(topics=['results']):
        if topic == 'parameters':
          # Store the parameter as strings
          curParams[msg.name.data] = msg.value.data
          
        if topic == 'results':
          if curFileId % frameSubsetRate <> 0:
            curFileId += 1
            continue

          rospy.loginfo('Processing ' + msg.image)
          self.grid = msg.grid
          blobFn = img2blobFn(msg.image, blobDir)
          self.fileIdMap[curFileId] = blobFn
          self.fileMap[blobFn] = curFileId
          self.processingTime.append(
            msg.scores.processing_time.data.to_sec())

          # Store the scores and best overlaps for this file
          self.scores = np.hstack((self.scores,
                                   ndBridge.matmsg_to_numpy(msg.confidence)))
          self.labels = np.hstack((self.labels,
                                   ndBridge.matmsg_to_numpy(msg.top_label)))
          self.notFishConf = np.hstack(
            (self.notFishConf,
             ndBridge.matmsg_to_numpy(msg.not_fish_confidence)))

          # Figure out the best overlaps
          image = cv2.imread(msg.image, 0)
          regions = DetectionUtils.GenerateSamplingRegions(
            image.shape, msg.grid)
          self.gridShape = DetectionUtils.GetGridShape(msg.grid, image.shape)
          bestOverlaps = np.zeros((self.scores.shape[0]))
          overlapLabels = np.zeros((self.scores.shape[0])) - 2
          curBlobId = 0
          for fishLoc in msg.regions:
            curLoc = (fishLoc.x_offset, fishLoc.y_offset, fishLoc.height,
                      fishLoc.width)
            self.blobList.append((curFileId, curBlobId))
            self.fishLoc.append(curLoc)
            overlaps = DetectionUtils.CalculateOverlapWithGrid(
                curLoc, regions)

            # Keep track of the maximum overlap for each location
            betterIdx = np.nonzero(bestOverlaps < overlaps)
            if len(betterIdx)[0] > 0:
              bestOverlaps[betterIdx] = overlaps[betterIdx]
              overlapLabels[betterIdx] = msg.labels[curBlobId]

            curBlobId += 1
              
            
          self.overlaps = np.hstack((self.overlaps, bestOverlaps))
          self.overlapLabels = np.hstack((self.overlapLabels, overlapLabels))

          curFileId += 1

    finally:
      curBag.close()
      self.params[bagFn] = curParams

def CreateClassifierParseObject(bagFile, options, cacheDir=None):
  # Determine the cache filename for the cached file
  hashVal = hashlib.md5()
  hashVal.update(VERSION)
  hashVal.update(bagFile)
  hashVal.update(struct.pack('!d', os.path.getmtime(bagFile)))
  hashVal.update(struct.pack('!i', options.frame_subset_rate))
  hashVal.update(os.path.abspath(options.blob_dir))
  hashVal.update(struct.pack('!d', options.overlap_thresh))

  cacheFile = None
  if cacheDir is not None:
    cacheFile = os.path.join(cacheDir, '%s.cparse' % hashVal.hexdigest())
    if os.path.exists(cacheFile):
      rospy.loginfo('Found cached ClassifierParse object. Opening %s' % cacheFile)
      return pickle.load(gzip.open(cacheFile, 'rb'))

  parseObj = ClassifierParse(inputBags, hashVal.hexdigest(),
                     options.frame_subset_rate,
                     options.blob_dir, options.overlap_thresh)

  if cacheDir is not None:
    outputFile = gzip.open(cacheFile, 'wb', 5)
    try:
      pickle.dump(parseObj, outputFile, 2)
    finally:
      outputFile.close()

  return parseObj

class VUStatistics:
  def __init__(self, options):
    self.VERSION = int(VERSION)
    self.options = options

    self.recall = []
    self.precision = []
    self.nWindows = []
    self.vuRisk = []
    self.classAccuracy = []
    self.classRecall = [] # Recall for the classifier being correct

    # Stats for the random resampling
    self.resampleRecall = []
    self.resampleVar = []
    self.resampleProcessingTime = []
    self.resampleNFish = [] # List of the number of fish in each bag
    self.resampleVuRisk = []

    # Stats for processing times
    self.vuProcessingTime = []

    nWindows = []
    times = []
    for line in open(options.class_timing_file):
      splitLine = line.strip().split(',')
      nWindows.append(int(splitLine[0]))
      times.append(float(splitLine[1]))
    self.ClassTiming = scipy.interpolate.interp1d(np.array(nWindows),
                                                  np.array(times),
                                                  kind='cubic')
    self.vuMaxClassTime = times[-1]
    self.vuMaxWindows = max(nWindows)
    
  def AppendStatsForVideo(self, vuParse, classParse):
    self.VerifyParseConsistency(vuParse, classParse)

    self.AppendResamplingStats(classParse)

    self.AppendVuStats(vuParse, classParse)

  def VerifyParseConsistency(vuParse, classParse):
    if vuParse.fileMap <> classParse.fileMap:
      LogAndRaiseError('Different files processed: \n%s\n%s' % (
        [x for x in vuParse.fileMap.keyiter() if x not in classParse.fileMap],
        [x for x in classParse.fileMap.keyiter() if x not in vuParse.fileMap]))
    
    if vuParse.scores.shape <> classParse.scores.shape:
      LogAndRaiseError('Shapes differ: %s vs. %s' % (vuParse.scores.shape,
                                                     classParse.scores.shape))

  def AppendResamplingStats(self, classParse):

    # First calculate a list of the number of positive classification
    # windows that overlap each fish.
    gridSize = np.cumprod(classParse.gridShape)
    CalcOverlapFunc = DetectionUtils.CalculateOverlapOfRegions
    Idx2Region = lambda x : DetectionUtils.GridIdx2Region(
      classParse.grid, x % gridSize, gridShape=classParse.gridShape)
    nPosWindows = [
      len([i for i in range(len(classParse.scores)) if
           classParse.scores[i] > options.confidence_thresh and
           classParse.scores[i] > classParse.notFishConf[i] and
           CalcOverlapFunc(fishLoc, Idx2Region(i)) > overlap_thresh]) for
      fishLoc in classParse.fishLoc]

    # Now calculate statistics
    nFish = len(nPosWindow)
    self.resampleNFish.append(nFish)
    fracWindows = np.exp(np.arange(0.0, 6, 0.05))
    nonZeroCounts = filter(lambda x: x>0, nPosWindows)
    nFound = np.array([sum([min(count/frac,1) for count in nonZeroCounts])
            for frac in fracWindows])
    self.resampleRecall = np.vstack((self.resampleRecall,
                                     np.divide(nFound, nFish)))
    self.resampleVar = np.vstack(
      (self.resampleVar,
       np.array([sum([self.GetVarianceOfOneFish(count, frac)
                      for count in nonZeroCounts])
                 for frac in fracWindows])))
    windowCount = np.array([self.vuMaxWindows/frac for frac in fracWindows])

    self.resampleProcessingTime = np.vstack((self.resampleProcessingTime,
                                             self.ClassTiming(windowCount)))

  def GetVarianceOfOneFish(self, fishCount, fracLeft):
    eCount = fishCount / fracLeft
    if eCount < 1:
      return eCount - eCount*eCount

    return 0

  def AppendVUStats(self, vuParse, classParse):
    # Initialize the stats
    precision = []
    recall = []
    classRecall = []
    nWindows = []
    vuRisk = []
    classAccuracy = []
    vuProcessingTime = []
    
    # All the regions that overlap with a fish
    groundTruthPos = vuParse.overlaps > options.overlap_thresh

    # All the regions that the classifier identifies as a fish
    classPos = np.logical_and(classParse.scores > classParse.notFishConf,
                              classParse.scores > options.confidence_thresh)

    # All the regions that the classifier correctly identifies the right fish
    classCorrect = np.logical_and(classPos,
                                  np.equal(classParse.labels,
                                           classParse.overlapLabels))

    # The maximum VUScore for each fish that is found by the classifier
    maxDetectScore = self._CalculateMaxVUScoreOnFish(vuParse, classPos)

    # The maximum VUScore for each fish that is classified as the
    # correct species
    maxCorrectScore = self._CalculateMaxVUScoreOnFish(vuParse, classCorrect)

    validVUScores = np.nonzero(np.isfinite(vuParse.scores))
    minVUScore = np.min(validVUScores)
    maxVUScore = np.max(validVUScores)
    for split in np.arange(minVUScore, maxVUScore,
                           (maxVUScore - minVUScore)/(options.n_splits+1)):
      vuPos = vuParse.scores >= split

      systemPos = np.logical_and(vuPos, classPos)
      nSystemPos = np.sum(systemPos)
      usefulPos = np.logical_and(classPos, groundTruthPos)

      # Calculate the precision
      if nSystemPos == 0:
        precision.append(float('nan'))
      else:
        precision.append(float(np.sum(np.logical_and(systemPos,
                                                     groundTruthPos))) /
                         nSystemPos)

      # Calculate the average number of windows passed by the vu filter
      nWindows.append(float(np.sum(vuPos)) / len(vuParse.fileMap))

      # Calculate the class accuracy
      if nSystemPos == 0:
        classAccuracy.append(float('nan'))
      else:
        classAccuracy.append(float(np.sum(np.logical_and(classCorrect,
                                                         systemPos))) /
                             nSystemPos)

      # Calculate the vuProcessingTime
      vuprocessingTime.append(np.mean(vuParse.processingTime))

      # Calculate the recall
      fishFound = maxDetectScore >= split
      nFishFound = float(np.sum(fishFound))
      if len(fishFound) == 0:
        recall.append(float('nan'))
      else:
        recall.append(nFishFound / len(fishFound))

      # Calculate the class recall
      fishCorrect = maxCorrectScore >= split
      nFishCorrect = float(np.sum(fishCorrect))
      if len(fishCorrect) == 0:
        classRecall.append(float('nan'))
      else:
        classRecall.append(nFishCorrect / len(fishCorrect))

    # Append the results
    self.recall = np.vstack((self.recall, recall))
    self.precision = np.vstack((self.precision, precision))
    self.nWindows = np.vstack((self.nWindows, nWindows))
    self.vuRisk = np.vstack((self.vuRisk, vuRisk))
    self.classAccuracy = np.vstack((self.classAccuracy, classAccuracy))
    self.classRecall = np.vstack((self.classRecall, classRecall))


  def _CalculateMaxVUScoreOnFish(self, vuParse, validScores, classParse):
    return [self._GetMaxScoreThatOverlaps(vuParse, validScores,
                                          vuScores.fishLoc[i],
                                          vuScores.blobList[i][0],
                                          classParse) for
            i in range(vuScores.fishLoc)]

  def _GetMaxScoreThatOverlaps(vuParse, validScores, fishLoc, fileId,
                               classParse):
    validRange = vuParse.fileRange(fileId)
    curScores = vuParse.scores[validRange[0]:validRange[1]]
    curScores = curScores[validScores[validRange[0]:validRange[1]]]

    OverlapFunc = DetectionUtils.CalculateOverlapOfRegions
    Idx2RegionFunc = lambda idx: DetectionUtils.GridIdx2Region(
      classParse.grid,
      idx,
      gridShape=classParse.gridShape)

    scores = [curScores[i] for i in range(len(curScores)) if
              OverlapFunc(fishLoc, Idx2RegionFunc(i)) >
              self.options.overlap_thresh]

    if len(scores) == 0:
      return -float('inf')

    return max(scores)
      
        
if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('--input', '-i', dest='input',
                    help='Glob specifying the input bags for vu estimators',
                    default=None)
  parser.add_option('--classifier_bags',
                    help='Glob specifying the bags that were the output from the classifier',
                    default=None)
  parser.add_option('--blob_dir', dest=None,
                    help='Directory where the blobs are that the humans labeled.')
  parser.add_option('--output', '-o', dest='output',
                    help='Filename of the output the pickle of the analysis stats',
                    default=None)
  parser.add_option('--overlap_thresh', default=0.5, type='float',
                    help='Minimum overlap threshold to accept a match')
  parser.add_option('--confidence_thresh', default=0.1, type='float',
                    help='Minimum confidence to find a fish.')
  parser.add_option('--frame_subset_rate', type='int', default=1,
                    help='In order to run quicker, you can only run the detector on a subset of the frames. This specifies how often to run the detector. So, for example if it is 5, it will run the detector once every 5 frames.')
  parser.add_option('--cache_dir',
                    default='/data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/vu_extraction/cache',
                    help='Directory to use to cache subcomponents in this data so that it can be computed faster')
  parser.add_option('--video_regex',
                    default='((([0-9][0-9]-){2})([0-9]{2}))',
                    help='Regex to extract the frame id and video id from filenames')
  parser.add_option('--class_timing_file',
                    default='',
                    help='File that specifies the timing for the classifier')
  parser.add_options('--n_splits', default=10, type='int'
                     help='Number of splits in possible vu scores to use')

  (options, args) = parser.parse_args()

  inputBags = FindInputBags(options.input, options.video_regex)
  classifierBags = FindInputBags(options.classifier_bags,
                                 options.video_regex)

  if classifierBags.keys() <> inputBags.keys():
    rospy.logfatal('The classifier and vu inputs are different: \n %s \n %s' %
                   (classifierBags.keys(), inputBags.keys()))
    return

  stats = VUStatistics()

  for videoId in classifierBags.keys():
    vuParse = VUAnalysisTools.CreateVUParseObject([inputBags[videoId]],
                                                  options, options.cache_dir,
                                                  allowInvalidScores=True)
    classifierParse = CreateClassifierParseObject(classifierBags[videoId],
                                                  options,
                                                  options.cache_dir)

    stats.AppendStatsForVideo(vuParse, classifierParse,
                              options.overlap_thresh)

  outputFile = open(options.output, 'wb')
  try:
    pickle.dump(stats, outputFile)
  finally:
    outputFile.close()
