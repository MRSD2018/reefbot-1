'''Helper for calculating the detector metrics.'''

import copy
import math
import numpy as np

class DetectorMetric:
  '''Abstract class to determine if a detection is consistent with the true location.'''
  def __init__(self, scoreThresh, doObjectMemory, widthRatio):
    self.scoreThresh = scoreThresh
    self.doObjectMemory = doObjectMemory
    self.widthRatio = widthRatio
    self.lastMsg = None
    self.detections = [] # List of (detection, score, matchingGroundTruth)
    self.groundTruths = [] # List of (groundTruth, maxScore)
    self.dataProcessed = 0
    self.nFrames = 0
    # Dictionaries of targetId -> firstInstance or (foundTime,score)
    self.targetInstance = {}
    self.targetFound = {}
    self.maxTargetScore = {}
    

  def GetPrecision(self):
    truePos = self.GetTruePos()
    falsePos = len([x for x in self.detections if
                    x[2] is None and x[1] > self.scoreThresh])
    if truePos > 0:
      return truePos / (truePos + self.GetFalsePos())
    return 0.0

  def GetRecall(self):
    truePos = self.GetTruePos()
    falseNeg = len([x for x in self.groundTruths if
                    x[1] is None or x[1] < self.scoreThresh])
    if truePos > 0:
      return truePos / (truePos + falseNeg)
    return 0.0

  def GetMissRate(self):
    return 1-self.GetRecall()

  def _GetFirstFoundAboveScore(self):
    retval = {}
    for k,v in self.targetFound.iteritems():
      for foundTime, score in v:
        if score > self.scoreThresh:
          retval[k] = foundTime
          break
    return retval

  def GetTargetMissRate(self):
    '''Returns the miss rate for unique targets.'''
    nTargets = len(self.targetInstance)
    targetsAboveScore = self._GetFirstFoundAboveScore()
    nTargetsFound = len([x for x in self.targetInstance if
                         x in targetsAboveScore])
    return 1.0 - float(nTargetsFound) / float(nTargets)

  def GetTargetMissWindow(self):
    '''For those targets found, how long does it take to find them once they are in the scene.'''
    targetsAboveScore = self._GetFirstFoundAboveScore()
    targetsFound = [x for x in self.targetInstance if
                    x in targetsAboveScore]
    tDiff = [targetsAboveScore[x] - self.targetInstance[x]
             for x in targetsFound]
    return np.mean(tDiff)

  def GetFalsePosPerFrame(self):
    return self.GetFalsePos() / self.nFrames

  def GetObjectPositionError(self):
    posErrorSum = 0.0
    for detection in self.detections:
      if detection[2] is not None and detection[1] > self.scoreThresh:
        posErrorSum = posErrorSum + self.CalcBoxDist(detection[0],
                                                     detection[2])

    if posErrorSum > 0:
      return posErrorSum / self.GetTruePos()
    return float('nan')

  def GetTruePos(self):
    return float(len([x for x in self.detections if
                      x[2] is not None and x[1] > self.scoreThresh]))

  def GetFalsePos(self):
    return float(len([x for x in self.detections if
                    x[2] is None and x[1] > self.scoreThresh]))

  def GetDataProcessed(self):
    if self.dataProcessed == 0:
      return 0
    return self.dataProcessed / self.nFrames

  def ProcessFrame(self, msg, dropFullFrames=False, imgSize=None):
    # Ignore when the full frame was sent to the detector
    if dropFullFrames and imgSize is not None and len(msg.roi) > 0:
      if msg.roi[0].width >= imgSize[0] and msg.roi[0].height >= imgSize[1]:
        return
      
    msgCopy = msg
    curTime = msgCopy.header.stamp.to_sec()

    # Add in detections from the last frame in regions we didn't run the
    # detector on this time.
    if self.doObjectMemory:
      if self.lastMsg is not None:
        msgCopy.detectorResponse.scores = list(msgCopy.detectorResponse.scores)
        nDetections = len(self.lastMsg.detectorResponse.detections)
        for i in range(nDetections):
          detection = self.lastMsg.detectorResponse.detections[i]
          score = self.lastMsg.detectorResponse.scores[i]
          inRoi = len(msgCopy.roi) == 0
          for roi in msgCopy.roi:
            if self.PointInRoi(roi,
                               detection.x_offset +
                               detection.width / 2,
                               detection.y_offset +
                               detection.height / 2):
              inRoi = True
              break
          if not inRoi:
            msgCopy.detectorResponse.detections.append(detection)
            msgCopy.detectorResponse.scores.append(score)
        
      self.lastMsg = msgCopy

    # Look for the best detection for each ground truth person
    for truePerson in msgCopy.groundTruth:
      if truePerson.annotationType < 0:
        continue

      # The first time this target is seen, record that
      if not truePerson.targetId in self.targetInstance:
        self.targetInstance[truePerson.targetId] = curTime
        self.maxTargetScore[truePerson.targetId] = -float('inf')
      
      bestScore = -float('inf')
      foundPerson = False
      nDetections = len(msgCopy.detectorResponse.detections)
      for i in range(nDetections):
        detection = msgCopy.detectorResponse.detections[i]
        score = msgCopy.detectorResponse.scores[i]
        if self.BoxesMatch(truePerson.bbox, detection) and score > bestScore:
          foundPerson = True
          bestScore = score

          # The first time the target is detected, record that
          if not truePerson.targetId in self.targetFound:
            self.targetFound[truePerson.targetId] = []
          if score > self.maxTargetScore[truePerson.targetId]:
            self.maxTargetScore[truePerson.targetId] = score
            self.targetFound[truePerson.targetId].append((curTime, score))
      if not foundPerson:
        bestScore = None
      self.groundTruths.append((truePerson.bbox, bestScore))

    # Look for the best ground truth person for each detection
    nDetections = len(msgCopy.detectorResponse.detections)
    for i in range(nDetections):
      detection = msgCopy.detectorResponse.detections[i]
      score = msgCopy.detectorResponse.scores[i]
      bestMatchScore = -float('inf')
      bestMatch = None
      for truePerson in msgCopy.groundTruth:
        if truePerson.annotationType < 0:
          continue
        
        curMatchScore = self.MatchingScore(truePerson.bbox, detection)
        if (self.BoxesMatch(truePerson.bbox, detection) and
            curMatchScore > bestMatchScore):
          bestMatchScore = curMatchScore
          bestMatch = truePerson.bbox
          break
      self.detections.append((detection, score, bestMatch))

    # Calculate how much data was sent to the person detector
    for roi in msgCopy.roi:
      self.dataProcessed = self.dataProcessed + roi.width * roi.height

    self.nFrames = self.nFrames + 1

  def CalcBoxDist(self, boxA, boxB):
    midYA = boxA.y_offset + boxA.height/2.0
    midYB = boxB.y_offset + boxB.height/2.0
    dRy = midYA - midYB

    midXA = boxA.x_offset + boxA.width/2.0
    midXB = boxB.x_offset + boxB.width/2.0
    dRx = midYA - midYB

    return math.sqrt(dRy*dRy + dRx*dRx)

  def PointInRoi(self, roi, x, y):
    if roi.height == 0 or roi.width == 0:
      return True
    return (roi.x_offset <= x and roi.y_offset <= y and
            x <= roi.x_offset + roi.width and
            y <= roi.y_offset + roi.height)

  def BoxesMatch(self, trueBox, detectedBox):
    return self.BoxesMatchImpl(self.NormalizeBoxRatio(trueBox),
                               self.NormalizeBoxRatio(detectedBox))

  def MatchingScore(self, trueBox, detectedBox):
    return self.MatchingScoreImpl(self.NormalizeBoxRatio(trueBox),
                                  self.NormalizeBoxRatio(detectedBox))

  def NormalizeBoxRatio(self, box):
    newWidth = box.height * self.widthRatio

    normBox = box
    normBox.x_offset = box.x_offset + (box.width - newWidth) / 2.0
    normBox.width = newWidth
    return normBox

  def BoxesMatchImpl(self, trueBox, detectedBox):
    raise NotImplementedError()

  def MatchingScoreImpl(self, trueBox, detectedBox):
    raise NotImplementedError()

  def GetMetricName(self):
    raise NotImplementedError()

class BoundingBoxOverlap(DetectorMetric):
  def __init__(self, scoreThresh, doObjectMemory, widthRatio, minOverlap):
    DetectorMetric.__init__(self, scoreThresh, doObjectMemory, widthRatio)
    self.minOverlap = minOverlap

  def MatchingScoreImpl(self, boxA, boxB):   
    midYA = boxA.y_offset + boxA.height/2.0
    midYB = boxB.y_offset + boxB.height/2.0
    dRy = boxA.height/2.0 + boxB.height/2.0 - abs(midYA - midYB)

    midXA = boxA.x_offset + boxA.width/2.0
    midXB = boxB.x_offset + boxB.width/2.0
    dRx = boxA.width/2.0 + boxB.width/2.0 - abs(midXA - midXB)

    if dRx < 0 or dRy < 0:
      # They do not overlap
      return False

    # They overlap so calculate the fraction overlap as the area that
    # overlaps over the area of the union.
    areaIntersection = float(dRy * dRx)
    areaUnion = (float(boxA.height*boxA.width) +
                 boxB.height*boxB.width -
                 areaIntersection)
    return areaIntersection / areaUnion
    

  def BoxesMatchImpl(self, boxA, boxB): 
    return self.MatchingScoreImpl(boxA, boxB) > self.minOverlap

  def GetMetricName(self):
    return 'Bounding Box Overlap > %f' % self.minOverlap

class MinimumBoundingBox(DetectorMetric):
  def __init__(self, scoreThresh, doObjectMemory, widthRatio):
    DetectorMetric.__init__(self, scoreThresh, doObjectMemory, widthRatio)

  def MatchingScoreImpl(self, boxA, boxB):
    if self.BoxesMatchImpl(boxA, boxB):
      return 1.0
    return 0.0

  def BoxesMatchImpl(self, trueBox, detectedBox):
    '''Make sure that the actual target reaches every side of the detected box.'''
    return (trueBox.y_offset <= detectedBox.y_offset and
            trueBox.x_offset <= detectedBox.x_offset and
            (trueBox.y_offset + trueBox.height) >
            (detectedBox.y_offset + detectedBox.height) and
            (trueBox.x_offset + trueBox.width) >
            (detectedBox.x_offset + detectedBox.width))

  def GetMetricName(self):
    return 'Minimum Bounding Box'
