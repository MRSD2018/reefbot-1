#!/usr/bin/python
'''Plots the correlation between visual utility values for two bags
'''
usage='PlotVUCorrelation.py [options] <bag1> <bag2>'

import roslib; roslib.load_manifest('hima_experiment')
import rospy
import rosbag
import os
import os.path
import re
import gc
import math
from pylab import *
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from optparse import OptionParser

class VUScores:
  '''Container for a filename->{region->(score, overlap)} structure.'''
  def __init__(self, bagFile, frameSubsetRate=1):
    rospy.loginfo('Reading bag %s' % (bagFile))
    self.scores = {}
    curFileId = 0
    bag = rosbag.Bag(bagFile)
    try:
      for topic, msg, t in bag.read_messages(topics=['results']):
        if topic == 'results':
          if curFileId % frameSubsetRate == 0:
            # Parse the result message
            region2Score = {}
            for i in xrange(len(msg.scores)):
              curRegion = (msg.regions[i].x_offset, msg.regions[i].y_offset,
                           msg.regions[i].height, msg.regions[i].width)
              region2Score[curRegion] = (msg.scores[i], msg.overlaps[i])
            self.scores[msg.image] = region2Score
          curFileId += 1

    finally:
      bag.close()

  def GetCorrelationPoints(self, otherScores):
    '''Returns (scoreThis, scoreOther, overlaps) where each is a numpy array of
    scores that match up between the scores in the object and those
    from the other object.
    '''
    scoreThis = []
    scoreOther = []
    overlaps = []
    for filename, region2ScoreA in self.scores.iteritems():
      if filename in otherScores.scores:
        region2ScoreB = otherScores.scores[filename]
        for region, valA in region2ScoreA.iteritems():
          scoreB, overlapB = region2ScoreB.get(region, (float('nan'), 0))
          if not math.isnan(scoreB):
            scoreThis.append(valA[0])
            scoreOther.append(scoreB)
            overlaps.append(overlapB)

    return (np.array(scoreThis), np.array(scoreOther), np.array(overlaps))

if __name__ == '__main__':
  parser = OptionParser(usage=usage)
  
  parser.add_option('--frame_subset_rate', type='int', default=1,
                    help='In order to run quicker, you can only run the detector on a subset of the frames. This specifies how often to run the detector. So, for example if it is 5, it will run the detector once every 5 frames.')
  parser.add_option('--bag_names', default=None,
                    help='Comma separated values for the bag names')

  (options, args) = parser.parse_args()

  bagA = args[0]
  bagB = args[1]

  bagNameA = os.path.basename(bagA)
  bagNameB = os.path.basename(bagB)
  if options.bag_names is not None:
    nameSplit = options.bag_names.split(',')
    bagNameA = nameSplit[0]
    bagNameB = nameSplit[1]

  scoresA = VUScores(bagA, options.frame_subset_rate)
  scoresB = VUScores(bagB, options.frame_subset_rate)

  pointsA, pointsB, overlap = scoresA.GetCorrelationPoints(scoresB)

  trueWindows = np.nonzero(overlap > 0.9)

  figure(1)
  # The 2D plot
  subplot(223)
  #scatter(pointsA, pointsB, s=3, hold=True)
  hexbin(pointsA, pointsB, bins='log', gridsize=50, cmap=cm.Blues, hold=True)
  scatter(pointsA[trueWindows], pointsB[trueWindows], s=5, c='r')
  xlabel(bagNameA)
  ylabel(bagNameB)
  subplot(221)
  #scatter(pointsA, overlap, s=3, hold=True)
  hexbin(pointsA, overlap, bins='log', gridsize=50, cmap=cm.Blues, hold=True)
  scatter(pointsA[trueWindows], overlap[trueWindows], s=5, c='r')
  ylabel('Grount Truth Overlap')
  subplot(224)
  #scatter(overlap, pointsB, s=3, hold=True)
  hexbin(overlap, pointsB, bins='log', gridsize=50, cmap=cm.Blues, hold=True)
  scatter(overlap[trueWindows], pointsB[trueWindows], s=5, c='r')
  xlabel('Ground Truth Overlap')

  # The 3D plot
  #fig2 = plt.figure(2)
  #axis2 = Axes3D(fig2)
  #axis2.scatter(pointsA, pointsB, overlap, s=5)
  #xlabel(os.path.basename(bagA))
  #ylabel(os.path.basename(bagB))
  #axis2.set_zlabel('Overlap fraction')

  show()
  
