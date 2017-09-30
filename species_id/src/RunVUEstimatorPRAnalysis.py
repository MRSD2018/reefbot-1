#!/usr/bin/python
'''Does the analysis for how accurate the visual utility estimation is for finding the human labeled fish.'''
usage='RunVUEstimatorPRAnalysis.py'

import roslib; roslib.load_manifest('species_id')
import rospy
import numpy as np
import scipy.interpolate
from optparse import OptionParser
import re
import os
import os.path
import cPickle as pickle
import math
import glob
import VUAnalysisTools

VERSION='3'

def FindInputBags(inputDir):
  return glob.glob(inputDir)
  #bagRe = re.compile(r'.*\.bag')
  #return [os.path.join(inputDir, x)
  #        for x in os.listdir(inputDir) if bagRe.match(x)]

def img2blobFn(imgFn, blobDir):
  imgRe = re.compile(r'(.*)\.(jpg|jpeg|bmp|png)')
  base = imgRe.match(os.path.basename(imgFn)).groups()[0]
  return os.path.join(blobDir, '%s.blob' % base)

class PRStatistics:
  def __init__(self, parse, overlapThresh):
    self.VERSION = VERSION

    self.recall = None
    self.precision = []

    for i in range(len(parse.bagIdx)):
      (curPrecision, self.recall) = self.CalculatePRForBag(parse, i, overlapThresh,
                                                           self.recall)
      self.precision.append(curPrecision)

    self.recall = np.array(self.recall)
    self.precision = np.array(self.precision)

    self.meanProcessingTime = np.mean(parse.processingTime)
    self.stdProcessingTime = np.std(parse.processingTime)
    self.stdErrorProcessingTime = self.stdProcessingTime / math.sqrt(len(parse.processingTime))

  def CalculatePRForBag(self, parse, bagI, overlapThresh, desiredRecallPoints=None):
    (p1, r1) = parse.bagIdx[bagI]
    if bagI + 1 == len(parse.bagIdx):
      p2 = len(parse.scores)
      r2 = len(parse.maxOverlapScore)
    else:
      (p2, r2) = parse.bagIdx[bagI + 1]

    curMaxOverlapScore = parse.maxOverlapScore[r1:r2]
    curScores = parse.scores[p1:p2]
    curOverlaps = parse.overlaps[p1:p2]

    splits = np.unique(
      curMaxOverlapScore[np.nonzero(np.isfinite(curMaxOverlapScore))])
    splits = splits[::-1]

    recall = []
    precision = []
    for split in splits:
      recall.append(len(np.nonzero(curMaxOverlapScore >= split)[0]) /
                         float(len(curMaxOverlapScore)))

      pos = curScores >= split

      precision.append(len(np.nonzero(
        np.logical_and(pos, curOverlaps > overlapThresh))[0]) /
                            float(len(np.nonzero(pos)[0])))

    recall.insert(0, 0.0)
    recall.append(1.0)
    precision.insert(0, precision[0])
    precision.append(0)

    # Do the interpolation
    if desiredRecallPoints is None:
      return (precision, recall)
    
    fInterp = scipy.interpolate.interp1d(recall, precision, 'linear')
    return(fInterp(desiredRecallPoints), desiredRecallPoints)
    

if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('--input', '-i', dest='input',
                    help='Glob specifying the input bags',
                    default=None)
  parser.add_option('--blob_dir', dest=None,
                    help='Directory where the blobs are that the humans labeled.')
  parser.add_option('--output', '-o', dest='output',
                    help='Filename of the output the pickle of the analysis stats',
                    default=None)
  parser.add_option('--overlap_thresh', default=0.5, type='float',
                    help='Minimum overlap threshold to accept a match')
  parser.add_option('--frame_subset_rate', type='int', default=1,
                    help='In order to run quicker, you can only run the detector on a subset of the frames. This specifies how often to run the detector. So, for example if it is 5, it will run the detector once every 5 frames.')
  parser.add_option('--cache_dir',
                    default='/data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/vu_extraction/cache',
                    help='Directory to use to cache subcomponents in this data so that it can be computed faster')

  (options, args) = parser.parse_args()

  inputBags = FindInputBags(options.input)

  parse = VUAnalysisTools.CreateVUParseObject(inputBags, options,
                                              options.cache_dir)

  stats = PRStatistics(parse, options.overlap_thresh)

  outputFile = open(options.output, 'wb')
  try:
    pickle.dump(stats, outputFile)
  finally:
    outputFile.close()
