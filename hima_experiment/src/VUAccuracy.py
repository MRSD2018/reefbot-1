#!/usr/bin/python
'''Tools to calculate the visual utility accuracy

The main class is the VUAccuracyCalculator that controls everything. It is designed to be built from the PlotVUAccuracy script.
'''

import roslib; roslib.load_manifest('hima_experiment')
import rospy
import rosbag
import numpy as np
import scipy.interpolate
import os
import os.path
import re
import gc
import cv2
import math
import cPickle as pickle
import cDetectionUtils
import DetectionUtils
import struct
import hashlib
import HimaDataLoader
from sensor_msgs.msg import RegionOfInterest
from optparse import OptionParser
import PlottingUtils

VERSION='7'

def PrepareCache(options):
  if not os.path.exists(options.cache_dir):
    os.makedirs(options.cache_dir)

def FindScaledHogBags(options):
  '''Find the scaled hog bags to process and returns (dataset, scalingFactor)->filename map.'''
  retval = {}
  scaledHogRe = re.compile(options.hog_scale_regex)
  for root, dirs, files in os.walk(options.root_results_dir):
    if os.path.samefile(root, options.root_results_dir):
      continue

    for filename in files:
      hogMatch = scaledHogRe.search(filename)
      if hogMatch:
        dataset = hogMatch.groups()[0]
        scalingFactor = float(hogMatch.groups()[1])
        retval[(dataset, scalingFactor)] = os.path.join(root, filename)

  return retval

def PackRegion(a):
  '''Packs a tuple (x,y,h,w) into a long integer.'''
  return a[0] | (a[1] << 16) | (a[2] << 32) | (a[3] << 48)

def UnpackRegion(a):
  '''Unpacks a long integer into a tuple (x,y,h,w).'''
  return (a & 0xFFFF,
          a >> 16 & 0xFFFF,
          a >> 32 & 0xFFFF,
          a >> 48 & 0xFFFF)

class VUScores:  
  def __init__(self):
    self.params = {} # bagFile -> {name -> value}
    self.results = {} # (fileId,region) -> (score, overlap)
    self.fileMap = {} # filename -> fileId
    self.fileIdMap = {} # fileId -> filename
    self.peopleLoc = {} # (imageFileId,personId)->PersonLocation
    self.processingTime = [] # list of all the processing times for each frame
    self.thresholdTimes = None # If changing the threshold changes the processing time, this is a tuple of (thresholds, runtimes)
    self.md5 = None # An identifier for this object

  def ParseBag(self, bagFiles, annotationsFiles, frameSubsetRate=1,
               doNMS=False, chosenThresh=None, overlapThresh=0.5,
               thresholdTimes=None, md5=None, imageIdRegex=None,
               frameSubsetOffset=1):
    self.thresholdTimes = thresholdTimes
    self.md5 = md5
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
        for topic, msg, t in bag.read_messages(topics=['parameters', 'results']):
          if topic == 'parameters':
            # Store the parameter as strings
            curParams[msg.name.data] = msg.value.data

          elif topic == 'results':
            # See if we should import this message
            imageId = int(imageIdRe.search(msg.image).groups()[0])
            if imageId % frameSubsetRate == frameSubsetOffset:

              # Parse the result message
              self.processingTime.append(msg.processing_time.data.to_sec())
              filenames.append(msg.image)
              regionScores = []
              for i in range(len(msg.scores)):
                curRegion = (msg.regions[i].x_offset, msg.regions[i].y_offset,
                             msg.regions[i].height, msg.regions[i].width)
                regionScores.append((curRegion, msg.scores[i]))

              if doNMS:
                regionScores = DetectionUtils.ApplyNMS(regionScores,
                                                       min(msg.scores),
                                                       chosenThresh,
                                                       overlapThresh)
              assert(len(regionScores) == len(msg.scores))
              for i in xrange(len(regionScores)):
                curRegion, curScore = regionScores[i]
                self.results[(curFileId, PackRegion(curRegion))] = (
                  curScore,
                  msg.overlaps[i])

              self.fileMap[msg.image] = curFileId
              self.fileIdMap[curFileId] = msg.image
            curFileId += 1

        fnPeopleLoc.update(HimaDataLoader.LoadPeopleLocations(
          annotationsFiles[curAnnotationsFile],
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

    return self

  def GetParameter(self, name):
    if name in self.params:
      return self.params[name]
    return None

  def GetScoreWithFn(self, filename, region):
    try:
      return self.results[(self.fileMap[filename], region)][0]
    except KeyError:
      return float('nan')
    
    #fileId = self.fileMap.get(filename, -1)
    #if fileId < 0:
    #  return float('nan')

    #return self.results.get((fileId, region), float('nan'))

  def iterFnResultsKey(self):
    '''An iterator that spits out (filename,region) keys.'''
    for fileId, region in self.results:
      yield (self.fileIdMap[fileId], region)

  def iterFnResultsItem(self):
    '''An iterator that spits out (score, overlap) items.'''
    for key, val in self.results.iteritems():
      yield ((self.fileIdMap[key[0]], key[1]), val)

def CreateVUScore(bagFiles, annotationsFiles, frameSubsetRate=1,
                  doNMS=False, chosenThresh=None, overlapThresh=0.5,
                  thresholdTimes=None, cacheDir=None, imageIdRegex=None,
                  frameSubsetOffset=0):
  # Determine the cache filename for the cached file
  hashVal = hashlib.md5()
  hashVal.update(VERSION)
  for bagFile in bagFiles:
    hashVal.update(bagFile)
    hashVal.update(struct.pack('!d', os.path.getmtime(bagFile)))
  for annotationFile in annotationsFiles:
    hashVal.update(annotationFile)
    hashVal.update(struct.pack('!d', os.path.getmtime(annotationFile)))
  hashVal.update(struct.pack('!i', frameSubsetRate))
  hashVal.update(struct.pack('!i', frameSubsetOffset))
  hashVal.update(struct.pack('?', doNMS))
  if chosenThresh is not None:
    hashVal.update(struct.pack('!d', chosenThresh))
  hashVal.update(struct.pack('!d', overlapThresh))
  if thresholdTimes is not None:
    for x in thresholdTimes[0]:
      hashVal.update(struct.pack('!d', x))
    for x in thresholdTimes[1]:
      hashVal.update(struct.pack('!d', x))
  
  cacheFile = None
  if cacheDir is not None:
    cacheFile = os.path.join(cacheDir, '%s.vuscore' % hashVal.hexdigest())
    if os.path.exists(cacheFile):
      rospy.loginfo('Found cached VUScores object. Opening %s' % cacheFile)
      return pickle.load(open(cacheFile, 'rb'))

  scoreObj = VUScores().ParseBag(bagFiles, annotationsFiles, frameSubsetRate,
                                 doNMS, chosenThresh, overlapThresh,
                                 thresholdTimes, hashVal.hexdigest(),
                                 imageIdRegex, frameSubsetOffset)
  if cacheDir is not None:
    outputFile = open(cacheFile, 'wb')
    try:
      pickle.dump(scoreObj, outputFile)
    finally:
      outputFile.close()

  return scoreObj

class ScoreMatrix:
  '''Encompases a matrix where each row is the score from an estimator and
  each column is a (filename, region) pair.
  '''
  def __init__(self, hogScore, vuScores, minOverlap, hogThresh,
               cacheDir=None, doVuNms=False, doPropagatePositives=False,
               offsetScores=None, hogOffsetScore=None):
    '''Build up the matrix

    Inputs:
    hogScore - A VUScores object for the hog results
    vuScores - A collection of the VUScores objects for all the estimators
    minOverlap - The minimum overlap to find the person
    hogThresh - The HOG threshold required
    cacheDir - Directory for storing cached data
    doVuNMS - Do non-maximal suppression on the vu scores?
    doPropagatePositives - Propages the positives from the previous frame?
    '''
    # The cache directory
    self.cacheDir = cacheDir

    # Store the non maximal suppression flag
    self.doVuNms = doVuNms

    # Store the popagatioin flag
    self.doPropagatePositives = doPropagatePositives
    
    # The number of images in the dataset
    self.nImages = float(len(hogScore.fileIdMap))
    
    # First grab the list of (fileId, box) groups
    self.regions = sorted(hogScore.results.keys())
    self.offsetRegions = None
    if hogOffsetScore is not None:
      self.offsetRegions = sorted(hogOffsetScore.results.keys())

    # Build an array for the ground truth. The overlap is used as the score
    self.groundTruth = np.array([hogScore.results[key][1] for
                                 key in self.regions])

    # Build an array for the HOG scores
    self.hogScores = np.array([hogScore.results[key][0] for
                               key in self.regions])
    self.offsetHogScores = None
    if hogOffsetScore is not None:
      self.offsetHogScores = np.array([hogOffsetScore.results[key][0] for
                                       key in self.offsetRegions])

    # Build the matrix of the scores for all the visual utility estimators
    self.matrix = np.array([self._GetMatrixLine(hogScore, vuScore,
                                                self.regions)
                            for vuScore in vuScores])
    self.offsetMatrix = None
    if hogOffsetScore is not None and offsetScores is not None:
      self.offsetMatrix = np.array([self._GetMatrixLine(hogOffsetScore,
                                                        offsetScore,
                                                        self.offsetRegions)
                                    for offsetScore in offsetScores])

    # Calculate the average processing time for each vu estimator
    self.processingTime = [sum(x.processingTime)/len(x.processingTime) for x
                           in vuScores]

    # Record the processing times for different thresholds for the vu
    # times that have it.
    self.thresholdTimes = [x.thresholdTimes for x in vuScores]

    if self.doVuNms:
      self.peopleLocs = [x.peopleLoc for x in vuScores]
      
    # Build a matrix of the maximum score where each person is found. Each
    # row is the score from a different estimator and each column is a
    # (filename, personId) pair.
    personMatrix = [self._GetPersonMatrixLine(vuScore, hogScore,
                                              hogThresh, minOverlap)
                    for vuScore in vuScores]
    personMatrix = np.array(personMatrix)
    self.personMatrix = personMatrix[:,:,1]
    self.peopleWindowCount = personMatrix[0,:,0]
      

    if np.max(np.isnan(self.matrix)) > 0:
      rospy.logwarn('There were %i regions in the hog results that were '
                    'not found in the visual utility estimator results.' %
                    np.sum(np.isnan(self.matrix)))

  def _GetMatrixLine(self, hogScore, vuScore, regions):
    cacheFile = None
    if self.cacheDir is not None:
      hashVal = hashlib.md5()
      hashVal.update(VERSION)
      hashVal.update(hogScore.md5)
      hashVal.update(vuScore.md5)
      cacheFile = os.path.join(self.cacheDir,
                               '%s.smatrix' % hashVal.hexdigest())
      if os.path.exists(cacheFile):
        return pickle.load(open(cacheFile, 'rb'))

    obj = [vuScore.GetScoreWithFn(hogScore.fileIdMap[fileId], region) for
           fileId, region in regions]

    if cacheFile is not None:
      outputFile = open(cacheFile, 'wb')
      try:
        pickle.dump(obj, outputFile)
      finally:
        outputFile.close()
      

    return obj

  def _GetPersonMatrixLine(self, vuScore, hogScore, hogThresh, minOverlap):
    cacheFile = None
    if self.cacheDir is not None:
      hashVal = hashlib.md5()
      hashVal.update(VERSION)
      hashVal.update(hogScore.md5)
      hashVal.update(vuScore.md5)
      hashVal.update(struct.pack('!d', hogThresh))
      hashVal.update(struct.pack('!d', minOverlap))
      cacheFile = os.path.join(self.cacheDir,
                               '%s.pmatrix' % hashVal.hexdigest())
      if os.path.exists(cacheFile):
        return pickle.load(open(cacheFile, 'rb'))

    MaxScoreFunc = self._GetMaxScoreThatOverlaps 
    line = [MaxScoreFunc(vuScore, peopleKey[0], personLoc,
                         minOverlap, hogScore, hogThresh)
            for peopleKey, personLoc in sorted(vuScore.peopleLoc.iteritems())]

    if cacheFile is not None:
      outputFile = open(cacheFile, 'wb')
      try:
        pickle.dump(line, outputFile)
      finally:
        outputFile.close()

    return line

  def _GetMaxScoreThatOverlaps(self, vuScore, imageFnId, personLoc,
                               minOverlap, hogScore, hogThresh):
    # Local variables to the functions to speed up the loop
    vuFileIdMap = vuScore.fileIdMap
    HogGetScoreWithFn = hogScore.GetScoreWithFn
    CalcOverlapTuple = personLoc.CalculateOverlapTuple
    
    scores = [score[0] for scoreKey, score in vuScore.results.iteritems() if
              scoreKey[0] == imageFnId and
              HogGetScoreWithFn(vuFileIdMap[scoreKey[0]], scoreKey[1]) >
              hogThresh and
              CalcOverlapTuple(UnpackRegion(scoreKey[1])) > minOverlap]

    if len(scores) == 0:
      return (0, -float('inf'))
    
    return (len(scores), max(scores))

  def GetStats(self, minOverlap, hogThresh, nSplits, hogTiming=None):
    '''Returns tuple of matrices (accuracy, precision, recall, errorRate,
    nWindows, processingTime, matthewsCoefficient).

    Each row in the matrix is for a different vuEstimator and each column
    is for a different split.
    '''
    accuracy = np.empty((self.matrix.shape[0], nSplits+1))
    precision = np.empty((self.matrix.shape[0], nSplits+1))
    recall = np.empty((self.matrix.shape[0], nSplits+1))
    errorRate = np.empty((self.matrix.shape[0], nSplits+1))
    nWindows = np.empty((self.matrix.shape[0], nSplits+1))
    processingTime = np.empty((self.matrix.shape[0], nSplits+1))
    matthewsCoefficient = np.empty((self.matrix.shape[0], nSplits+1))
    vuNegPrecision = np.empty((self.matrix.shape[0], nSplits+1))
    vuPrecision =  np.empty((self.matrix.shape[0], nSplits+1))
    vuRecall  = np.empty((self.matrix.shape[0], nSplits+1))
    
    peoplePos = self.groundTruth > minOverlap
    hogPos = self.hogScores > hogThresh
    offsetHogPos = None
    if self.offsetHogScores is not None:
      offsetHogPos = self.offsetHogScores > hogThresh
    
    minScores = np.min(self.matrix, 1)
    maxScores = np.max(self.matrix, 1)
    splits = np.array([np.arange(minScores[i], maxScores[i],
                                 (maxScores[i] - minScores[i])/(nSplits+1))
                       for i in range(len(minScores))])      
    for row in range(self.matrix.shape[0]):
      timingInterp = None
      if self.thresholdTimes[row] is not None:
        timingInterp = scipy.interpolate.interp1d(self.thresholdTimes[row][0],
                                                  self.thresholdTimes[row][1])
      for col in range(len(splits[row])):
        if col >= accuracy.shape[1]:
          break
        offsetRow = None
        if self.offsetMatrix is not None:
          offsetRow = self.offsetMatrix[row]
        
        vuPos = self._GetVuPos(self.matrix[row], splits[row][col],
                               minScores[row], hogPos, offsetRow,
                               offsetHogPos)
        pos = np.logical_and(vuPos, hogPos)
        hogPeoplePos = np.logical_and(hogPos, peoplePos)
        truePos = float(np.sum(np.logical_and(pos, peoplePos)))
        falsePos = float(np.sum(np.logical_and(pos,
                                               np.logical_not(peoplePos))))
        falseNeg = float(np.sum(np.logical_and(np.logical_not(pos),
                                               peoplePos)))
        trueNeg = float(np.sum(np.logical_and(np.logical_not(pos),
                                              np.logical_not(peoplePos))))

        # Change accuracy to the Bayes risk with the cost of a false
        # positive reduced by the probability that the hog detector
        # will accept it.
        #accuracy[row][col] = float(np.sum(pos)) / np.sum(np.logical_or(vuPos,
        #                                                        hogPos))
        accuracy[row][col] = (np.sum(
                                np.logical_and(np.logical_not(vuPos),
                                               hogPeoplePos)) +
                              np.sum(hogPeoplePos)/len(hogPeoplePos) *
                              np.sum(
                                np.logical_and(vuPos,
                                               np.logical_not(hogPeoplePos))))
        
        if np.sum(pos) == 0:
          precision[row][col] = float('nan')
        else:
          precision[row][col] = truePos / (truePos + falsePos)

        if np.sum(np.logical_not(vuPos)) == 0:
          vuNegPrecision[row][col] = float('nan')
        else:
          vuNegPrecision[row][col] = float(np.sum(
            np.logical_and(np.logical_not(vuPos),
                           np.logical_not(np.logical_and(hogPos,
                                                         peoplePos)))))\
                           / np.sum(np.logical_not(vuPos))

        vuTruePos = float(np.sum(np.logical_and(vuPos, hogPeoplePos)))
        if np.sum(vuPos) == 0:
          vuPrecision[row][col] = float('nan')
        else:
          vuPrecision[row][col] = vuTruePos / float(np.sum(vuPos))

        if np.sum(hogPeoplePos) == 0:
          vuRecall[row][col] = float('nan')
        else:
          vuRecall[row][col] = vuTruePos / float(np.sum(hogPeoplePos))
        
        errorRate[row][col] = float(np.sum(pos <> peoplePos)) / len(pos)
        nWindows[row][col] = float(np.sum(vuPos)) / self.nImages

        if timingInterp is None:
          processingTime[row][col] = hogTiming(nWindows[row][col]) + \
                                     self.processingTime[row]
        else:
          processingTime[row][col] = hogTiming(nWindows[row][col]) + \
                                     timingInterp(splits[row][col])

        if ((truePos + falsePos == 0) or (truePos + falseNeg == 0) or
            (trueNeg + falsePos == 0) or (trueNeg + falseNeg == 0)):
          matthewsCoefficient[row][col] = 0
        else:
          matthewsCoefficient[row][col] = (
            (truePos * trueNeg - falsePos * falseNeg) /
            math.sqrt((truePos + falsePos) * (truePos + falseNeg) *
                      (trueNeg + falsePos) * (trueNeg + falseNeg)))

        # Recall is more complicated because we're looking at the
        # recall of people, not of windows.
        if self.doVuNms:
          peopleFound = []
          for personKey, personLoc in self.peopleLocs[row].iteritems():
            fileId = personKey[0]
            CalcOverlapFunc = personLoc.CalculateOverlapTuple
            curMatchingWindows = np.array([
              CalcOverlapFunc(UnpackRegion(self.regions[posIdx][1])) >
              minOverlap
              for posIdx in np.nonzero(pos)[0] if
              self.regions[posIdx][0] == fileId])
            peopleFound.append(np.any(curMatchingWindows))
        else:
          peopleFound = self.personMatrix[row] >= splits[row][col]    
        nPeopleFound = float(np.sum(peopleFound))
        if len(peopleFound) == 0:
          recall[row][col] = float('nan')
        else:
          recall[row][col] = nPeopleFound / len(peopleFound)
        
    return (accuracy, precision, recall, errorRate, nWindows, processingTime,
            matthewsCoefficient, vuNegPrecision, vuPrecision, vuRecall)

  def _GetVuPos(self, scoreRow, vuThresh, minScore, hogPos,
                offsetScoreRow=None, offsetHogPos=None):
    '''Retuns an indicator array which is 1 for each of the entries
    identified by the visual utility score.'''
    if self.doVuNms or self.doPropagatePositives:
      # We need to do non maximal suppression so for each file, do the
      # non maximal suppression. To do this, we step through the score
      # row breaking for each file.
      results = []
      startIdx = 0
      curFile = None
      lastPos = None
      curScores = None
      for i in xrange(len(self.regions)+1):
        if i == len(self.regions) or self.regions[i][0] <> curFile:
          # At the end of the entires for this file, so process them.
          if curFile is not None:
            curScores = [(UnpackRegion(self.regions[x][1]), scoreRow[x])
                         for x in xrange(startIdx, i)]
            if self.doVuNms:
              # Do non-maximal suppression
              curScores = DetectionUtils.ApplyNMS(curScores, minScore,
                                                  vuThresh)

            curScores = [x[1] >= vuThresh for x in curScores]
            if self.doPropagatePositives:
              # Take the positive hits from the last frame and flag
              # them past the visual utility filter. Does not work with NMS
              if offsetHogPos is not None:
                if i <= len(self.offsetRegions):
                  lastPos = np.logical_and(offsetHogPos[startIdx:i],
                                           [offsetScoreRow[x] >= vuThresh for
                                            x in xrange(startIdx, i)])
                else:
                  lastPos = None
              if lastPos is not None:
                curScores = np.logical_or(curScores, lastPos)
            
            results.extend(curScores)
            lastPos = np.logical_and(curScores, hogPos[startIdx:i])
          if i == len(self.regions):
            curFile = None
          else:
            curFile = self.regions[i][0]
          startIdx = i
      return results
    else:
      return scoreRow >= vuThresh

def GetMaxScoreThatOverlapsHog(hogScore, imageFnId, personLoc, minOverlap,
                               hogThresh, validKeys):
  CalcOverlapTuple = personLoc.CalculateOverlapTuple
  hogResults = hogScore.results

  scoreBox = ((hogResults[key][0], key[1]) for key in validKeys if
            imageFnId == key[0])
  scores = [score for score, box in scoreBox if
            score > hogThresh and
            CalcOverlapTuple(UnpackRegion(box)) > minOverlap]

  if len(scores) == 0:
      return -float('inf')
    
  return max(scores)

def CalculatePRCurve(vuScores, minOverlap):
  # First grab the list of (fileId, box) groups
  regions = vuScores.results.keys()

  if len(regions) == 0:
    return (1.0, float('nan'), float('nan'), float('nan'), 0,
            float('nan'), 0)

  # Build an array for the ground truth. The overlap is used as the score
  groundTruth = np.array([vuScores.results[key][1] for key in regions])
  peoplePos = groundTruth > minOverlap

  # Build an array for the actual scores
  scores = np.array([vuScores.results[key][0] for key in regions])

  # Build a list of the maximum hog score around each person in the dataset
  regions = set(regions)
  peopleScores = np.array(
    [GetMaxScoreThatOverlapsHog(vuScores, peopleKey[0], personLoc,
                 minOverlap, float('-inf'), regions)
     for peopleKey, personLoc in vuScores.peopleLoc.iteritems()])

  minScore = min(scores)
  maxScore = max(scores)
  steps = float(min(len(scores), 100))
  precision = []
  recall = []
  for thresh in np.arange(minScore, maxScore, (maxScore - minScore)/steps):
    vuPos = scores > thresh
    truePos = float(np.sum(np.logical_and(vuPos, peoplePos)))
    falsePos = float(np.sum(np.logical_and(vuPos,
                                           np.logical_not(peoplePos))))
    if truePos + falsePos == 0:
      precision.append(float('nan'))
    else:
      precision.append(truePos / (truePos + falsePos))

    
    recall.append(float(np.sum(peopleScores > thresh)) /
                  len(peopleScores))

  # Now sort the precision and recall
  recall = np.array(recall)
  precision = np.array(precision)
  sortIdx = np.argsort(recall)

  return (precision[sortIdx], recall[sortIdx])

def CalculateRecall(scores, minOverlap, hogThresh, regions, cacheDir=None):
  cacheFile = None
  if cacheDir is not None:
      hashVal = hashlib.md5()
      hashVal.update(VERSION)
      hashVal.update(scores.md5)
      hashVal.update(struct.pack('!d', minOverlap))
      hashVal.update(struct.pack('!d', hogThresh))
      for region in regions:
        hashVal.update(scores.fileIdMap[region[0]])
        hashVal.update(struct.pack('!q', region[1]))
      cacheFile = os.path.join(cacheDir,
                               '%s.recall' % hashVal.hexdigest())
      if os.path.exists(cacheFile):
        return pickle.load(open(cacheFile, 'rb'))
      
  
  # Recall is funny because we need it to be the recall of the people,
  # not the number of windows that overlap with the people.
  setRegions = set(regions)
  peopleScores = np.array(
    [GetMaxScoreThatOverlapsHog(scores, peopleKey[0], personLoc,
                 minOverlap, hogThresh, setRegions)
     for peopleKey, personLoc in scores.peopleLoc.iteritems()])
  recallVal = float(np.sum(np.isfinite(peopleScores))) / \
              len(peopleScores)

  if cacheFile is not None:
    outputFile = open(cacheFile, 'wb')
    try:
      pickle.dump(recallVal, outputFile)
    finally:
      outputFile.close()

  return recallVal

def GetValidRegions(hogScore, strideSteps, scaleSteps, cacheDir):
  cacheFile = None
  if cacheDir is not None:
    hashVal = hashlib.md5()
    hashVal.update(VERSION)
    hashVal.update(hogScore.md5)
    hashVal.update(struct.pack('!i', strideSteps))
    hashVal.update(struct.pack('!i', scaleSteps))

    cacheFile = os.path.join(cacheDir, '%s.regions' % hashVal.hexdigest())

    if os.path.exists(cacheFile):
      return pickle.load(open(cacheFile, 'rb'))

  # First grab the list of (fileId, box) groups
  regions = sorted(hogScore.results.keys())

  if len(regions) == 0:
    return regions

  # Filter the regions based on the stride and scale step
  cvImage = cv2.imread(hogScore.fileIdMap.itervalues().next())
  curStride = min([box & 0xFFFF for fileId, box in regions if
                   box & 0xFFFF > 0])
  validBoxes = DetectionUtils.GenerateSamplingRegions(
    cvImage,
    min([box >> 48 & 0xFFFF for fileId, box in regions]),
    min([box >> 32 & 0xFFFF for fileId, box in regions]),
    curStride * strideSteps,
    math.pow(1.10, scaleSteps), # scaleStride
    doTuple=True)
  validBoxes = set(map(lambda x: PackRegion(x), validBoxes))

  rospy.logwarn('The scale increment must be 1.1 in this HOG bag. I hard coded it!')

  regions = filter(lambda key: key[1] in validBoxes, regions)

  if cacheFile is not None:
    outputFile = open(cacheFile, 'wb')
    try:
      pickle.dump(regions, outputFile)
    finally:
      outputFile.close()

  return regions

def CalculateHOGStatistics(hogScore, minOverlap, hogThresh, strideSteps=1,
                           scaleSteps=1, cacheDir=None):
  accuracy = 1.0

  # First grab the list of (fileId, box) groups
  regions = GetValidRegions(hogScore, strideSteps, scaleSteps, cacheDir)

  if len(regions) == 0:
    return (1.0, float('nan'), float('nan'), float('nan'), 0,
            float('nan'), 0, float('nan'), float('nan'), float('nan'))

  # Build an array for the ground truth. The overlap is used as the score
  groundTruth = np.array([hogScore.results[key][1] for key in regions])

  # Build an array for the HOG scores
  hogScores = np.array([hogScore.results[key][0] for key in regions])

  peoplePos = groundTruth > minOverlap
  hogPos = hogScores > hogThresh

  truePos = float(np.sum(np.logical_and(hogPos, peoplePos)))
  falsePos = float(np.sum(np.logical_and(hogPos, np.logical_not(peoplePos))))
  falseNeg = float(np.sum(np.logical_and(np.logical_not(hogPos), peoplePos)))
  trueNeg = float(np.sum(np.logical_and(np.logical_not(hogPos),
                                        np.logical_not(peoplePos))))

  if np.sum(hogPos) == 0:
    precision = float('nan')
  else:
    precision = truePos / (truePos + falsePos)

  if np.sum(np.logical_not(hogPos)) == 0:
    negPrecision = float('nan')
  else:
    negPrecision = trueNeg / (trueNeg + falseNeg)

  vuPrecision = truePos / len(regions)

  if np.sum(peoplePos) == 0:
    vuRecall = float('nan')
  else:
    # We're going to estimate the recall here as the recall of finding
    # windows by scaling it down by the fraction of windows that were
    # actually looked at.
    vuRecall = float(len(regions)) / len(hogScore.results)

  if ((truePos + falsePos == 0) or (truePos + falseNeg == 0) or
      (trueNeg + falsePos == 0) or (trueNeg + falseNeg == 0)):
    matthewsCoefficient = 0
  else:
    matthewsCoefficient = (
      (truePos * trueNeg - falsePos * falseNeg) /
      math.sqrt((truePos + falsePos) * (truePos + falseNeg) *
                (trueNeg + falsePos) * (trueNeg + falseNeg)))
  
  errorRate = float(np.sum(hogPos <> peoplePos)) / len(hogPos)
  nWindows = float(len(hogPos)) / float(len(hogScore.fileIdMap))
  processingTime = sum(hogScore.processingTime)/len(hogScore.processingTime)

  recall = CalculateRecall(hogScore, minOverlap, hogThresh, regions, cacheDir)

  return (accuracy, precision, recall, errorRate, nWindows,
          matthewsCoefficient, processingTime, negPrecision, vuPrecision,
          vuRecall)

class VUAccuracyCalculator:
  def __init__(self, options):
    self.VERSION = int(VERSION) # Version number if we change how the stats are calculated.
    self.options = options # To record all the settings
    self.accuracy = []
    self.precision = []
    self.recall = []
    self.errorRate = []
    self.groundTruths = []
    self.resampleHogStats = []
    self.scaledHogStats = []
    self.nWindows = []
    self.processingTime = []
    self.matthewsCoefficients = []
    self.hogPRCurves = []
    self.vuNegPrecision = []
    self.vuRecall = []
    self.vuPrecision = []
    self.peopleWindowCount = {} # (imageFile, personId)-># positive windows

    PrepareCache(options)

    garb, self.hogProcessingTime, hogTiming = \
          PlottingUtils.ParseHogTiming(options)

    annotationsMap = PlottingUtils.ParseAnnotationFiles(options)
    bags, datasets, vuTypes = PlottingUtils.FindVUBags(options)
    
    scaledHogBags = FindScaledHogBags(options)
    self.nVUEstimators = len(vuTypes)
    if options.do_resampling:
      self.nVUEstimators += 1
    scalingFactors = sorted(set([scalingFactor for x, scalingFactor in
                                 scaledHogBags.iterkeys()]))
    timingData = PlottingUtils.FindTimingData(vuTypes, options)
  
    hogBag = CreateVUScore([bags[(options.hog_name, x)] for x in datasets],
                           [annotationsMap[x] for x in datasets],
                           options.frame_subset_rate,
                           options.do_nms,
                           options.hog_thresh,
                           options.min_overlap,
                           cacheDir=options.cache_dir,
                           imageIdRegex=options.imageid_regex)
    hogOffsetBag = None
    if options.frame_subset_rate >= 2:
      hogOffsetBag = hogBag
      hogBag = CreateVUScore([bags[(options.hog_name, x)] for x in datasets],
                             [annotationsMap[x] for x in datasets],
                             options.frame_subset_rate,
                             options.do_nms,
                             options.hog_thresh,
                             options.min_overlap,
                             cacheDir=options.cache_dir,
                             imageIdRegex=options.imageid_regex,
                             frameSubsetOffset=1)

    for vuType in vuTypes:
      offsetVuBags = None
      vuBags = [CreateVUScore([bags[(vuType, x)] for x in datasets],
                              [annotationsMap[x] for x in datasets],
                              options.frame_subset_rate,
                              thresholdTimes=timingData[vuType],
                              cacheDir=options.cache_dir,
                              imageIdRegex=options.imageid_regex)]
      if options.frame_subset_rate >= 2:
        offsetVuBags = vuBags
        vuBags = [CreateVUScore([bags[(vuType, x)] for x in datasets],
                                [annotationsMap[x] for x in datasets],
                                options.frame_subset_rate,
                                thresholdTimes=timingData[vuType],
                                cacheDir=options.cache_dir,
                                imageIdRegex=options.imageid_regex,
                                frameSubsetOffset=1)]
      
      scoreMatrix = ScoreMatrix(hogBag, vuBags, options.min_overlap,
                                options.hog_thresh, options.cache_dir,
                                options.do_vu_nms, options.prop_pos,
                                offsetVuBags, hogOffsetBag)
      (curAccuracy, curPrecision, curRecall, curErrorRate, curNWindows,
       curProcessingTime, curMatCoefs, curVuNegPrecision,
       curVuPrecision, curVuRecall) = \
       scoreMatrix.GetStats(options.min_overlap, options.hog_thresh,
                            20, hogTiming)
      self.accuracy.append(curAccuracy)
      self.precision.append(curPrecision)
      self.recall.append(curRecall)
      self.errorRate.append(curErrorRate)
      self.nWindows.append(curNWindows)
      self.processingTime.append(curProcessingTime)
      self.matthewsCoefficients.append(curMatCoefs)
      self.vuNegPrecision.append(curVuNegPrecision)
      self.vuPrecision.append(curVuPrecision)
      self.vuRecall.append(curVuRecall)

      if len(self.peopleWindowCount) == 0:
        self.peopleWindowCount = scoreMatrix.peopleWindowCount

      del vuBags
      del scoreMatrix
      gc.collect()

    # Do the calculator for resampling the boxes
    if options.do_resampling:
      for scaleStep in range(1,8,2):
        for strideStep in range(1,8,2):
          self.resampleHogStats.append(
            CalculateHOGStatistics(hogBag,
                                   options.min_overlap,
                                   options.hog_thresh,
                                   strideStep,
                                   scaleStep,
                                   options.cache_dir))

      self.resampleHogStats.extend([tuple([float('nan') for x in range(10)])
                                    for y in range(curAccuracy.shape[1] -
                                                   len(self.resampleHogStats))])
      resampleProcessingTime = hogTiming([x[4] for
                                          x in self.resampleHogStats])
      sortIdx = np.argsort(resampleProcessingTime)
      self.accuracy.append([[self.resampleHogStats[x][0] for x in sortIdx]])
      self.precision.append([[self.resampleHogStats[x][1] for x in sortIdx]])
      self.recall.append([[self.resampleHogStats[x][2] for x in sortIdx]])
      self.errorRate.append([[self.resampleHogStats[x][3] for x in sortIdx]])
      self.nWindows.append([[self.resampleHogStats[x][4] for x in sortIdx]])
      self.matthewsCoefficients.append(
        [[self.resampleHogStats[x][5] for x in sortIdx]])
      self.processingTime.append([np.sort(resampleProcessingTime)])
      self.vuNegPrecision.append([[self.resampleHogStats[x][7] for
                                   x in sortIdx]])
      self.vuRecall.append([[self.resampleHogStats[x][9] for x in sortIdx]])
      self.vuPrecision.append([[self.resampleHogStats[x][8]
                                for x in sortIdx]])

    # Calculate the statistics for no VU filter
    self.groundTruths.append(
      CalculateHOGStatistics(hogBag,
                             options.min_overlap,
                             options.hog_thresh,
                             cacheDir=options.cache_dir))
    self.hogPRCurves.append(CalculatePRCurve(hogBag, options.min_overlap))
    del hogBag
    gc.collect()

    # Now cycle through all the scaled hog bags to get a baseline
    # assuming resizing.
    if options.do_scaling:
      for scalingFactor in scalingFactors:
        scaledHogBag = CreateVUScore(
          [scaledHogBags[(x, scalingFactor)] for x in datasets],
          [annotationsMap[x] for x in datasets],
          options.frame_subset_rate,
          options.do_nms,
          options.hog_thresh,
          options.min_overlap,
          cacheDir=options.cache_dir,
          imageIdRegex=options.imageid_regex)
        self.scaledHogStats.append(
          CalculateHOGStatistics(scaledHogBag,
                                 options.min_overlap,
                                 options.hog_thresh,
                                 cacheDir=options.cache_dir))

        del scaledHogBag
        gc.collect()

    self.accuracy = np.concatenate(self.accuracy, 0)
    self.precision = np.concatenate(self.precision, 0)
    self.recall = np.concatenate(self.recall, 0)
    self.errorRate = np.concatenate(self.errorRate, 0)
    self.nWindows = np.concatenate(self.nWindows, 0)
    self.processingTime = np.concatenate(self.processingTime, 0)
    self.matthewsCoefficients = np.concatenate(self.matthewsCoefficients, 0)
    self.vuNegPrecision = np.concatenate(self.vuNegPrecision,0)
    self.vuPrecision = np.concatenate(self.vuPrecision,0)
    self.vuRecall = np.concatenate(self.vuRecall,0)
    
    self.vuTypes = vuTypes

if __name__ == '__main__':
  parser = OptionParser()

  parser.add_option('--imageid_regex', default='([0-9]+)\.((png)|(jpg)|(bmp))',
                    help='Regex to extract the image id from a filename')
  parser.add_option('--hog_scale_regex',
                    default='vu_HOGDetector_([0-9A-Za-z]+)_scale([0-9\.]+)\.bag',
                    help='Regex to match scaled HOG runs.')
  parser.add_option('--min_overlap', type='float',
                    help='Minimum fraction overlap for a target to be correct',
                    default=0.5)
  parser.add_option('--hog_thresh', type='float',
                    help='Minimum HOG value required to label a hit',
                    default=0.0)
  parser.add_option('--frame_subset_rate', type='int', default=1,
                    help='In order to run quicker, you can only run the detector on a subset of the frames. This specifies how often to run the detector. So, for example if it is 5, it will run the detector once every 5 frames.')
  parser.add_option('--do_nms', action='store_true', default=False,
                    help='Do non maximal suppression of the HOG scores?')
  parser.add_option('--do_vu_nms', action='store_true', default=False,
                    help='Do non maximal suppression of the VU scores?')
  parser.add_option('--prop_pos', action='store_true', default=False,
                    help='Propagate hits from one frame to the next?')
  parser.add_option('--skip_scaling', action='store_false', default=True,
                    dest='do_scaling')
  parser.add_option('--skip_resampling', action='store_false', default=True,
                    dest='do_resampling')
  parser.add_option('--cache_dir',
                    default='/data/mdesnoye/pedestrian/vu_estimation/eth/cache',
                    help='Directory to use to cache subcomponents in this data so that it can be computed faster')
  parser.add_option('--output_data', '-o', default='VUAccuracy.stats',
                    help='Filename where the stats will be pickled to')

  parser = PlottingUtils.AddCommandLineOptions(parser)

  (options, args) = parser.parse_args()

  stats = VUAccuracyCalculator(options)
  rospy.loginfo('Saving calculated statistics to: %s' % options.output_data)
  dirName = os.path.dirname(options.output_data)
  if not os.path.exists(dirName) and dirName <> '':
    os.makedirs(dirName)
  outputFile = open(options.output_data, 'wb')
  try:
    pickle.dump(stats, outputFile)
  finally:
    outputFile.close()
