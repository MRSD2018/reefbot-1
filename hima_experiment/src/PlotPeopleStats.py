#!/usr/bin/python
'''Plots some graphs about the statistics of the people and the boxes that find them.
'''
import roslib; roslib.load_manifest('hima_experiment')
import rospy
import rosbag
from optparse import OptionParser
import HimaDataLoader
from pylab import *
import numpy as np
import math
import PlottingUtils

class VUScores:
  def __init__(self, options):
    self.personLoc = {} # (imageFile, personId)->personLocation
    self.windowCount = {} # (imageFile, personId)-># of positive windows
    self.hogScores = {} # imageFile->[(region,score)] if score >= hog_thresh
    self.nWindows = 0 # Number of windows sampled per frame

    filenames = []
    curFileId = 0
    rospy.loginfo('Reading bag %s' % options.hog_file)
    bag = rosbag.Bag(options.hog_file)
    try:
      for topic, msg, t in bag.read_messages(topics=['results']):
        if topic == 'results':
          if curFileId % options.frame_subset_rate == 0:
            filenames.append(msg.image)
            curScores = []

            self.nWindows = len(msg.scores)

            for i in xrange(self.nWindows):
              if msg.scores[i] > options.hog_thresh:
                curRegion = (msg.regions[i].x_offset, msg.regions[i].y_offset,
                             msg.regions[i].height, msg.regions[i].width)
                curScores.append((curRegion, msg.scores[i]))
            self.hogScores[msg.image] = curScores

          curFileId += 1

      self.personLoc = HimaDataLoader.LoadPeopleLocations(options.annotations,
                                                          filenames)
          
    finally:
      bag.close()

    rospy.loginfo('Counting the number of windows around each person')
    for key, value in self.personLoc.iteritems():
      self.windowCount[key] = self._CountOverlapWindows(
        options.overlap_thresh,
        value,
        self.hogScores[key[0]])

  def _CountOverlapWindows(self, overlapThresh, loc, hogScores):
    windows = [region for region, score in hogScores if
               loc.CalculateOverlapTuple(region) >= overlapThresh]

    return len(windows)

def GetVarianceOfOnePerson(peopleCount, fracLeft):
  eCount = peopleCount / fracLeft
  if eCount < 1:
    return eCount - eCount*eCount

  return 0

if __name__ == '__main__':
  parser = OptionParser()

  parser.add_option('--hog_file', default=None,
                    help='File with the HOG measurements.')
  parser.add_option('--hog_thresh', default=0.0, type='float',
                    help='Threshold for the HOG detector')
  parser.add_option('--annotations', default=None,
                    help='Annotations file')
  parser.add_option('--frame_subset_rate', type='int', default=1,
                    help='In order to run quicker, you can only run the detector on a subset of the frames. This specifies how often to run the detector. So, for example if it is 5, it will run the detector once every 5 frames.')
  parser.add_option('--overlap_thresh',  default=0.5, type='float',
                    help='Overlap threshold')
  parser.add_option('--hog_timing_file',
                    default='/data/mdesnoye/pedestrian/vu_estimation/eth/hog/hog_timing.txt',
                    help='File that specifies the timing for the hog')
  (options, args) = parser.parse_args()

  scores = VUScores(options)

  # Plot a histogram of the number of positive windows per person
  nonZeroCounts = filter(lambda x: x>0, scores.windowCount.values())
  figure()
  hist(nonZeroCounts, 50, histtype='step')

  # Plot a histogram of the size of people
  figure()
  hist([math.sqrt(x.Area()) for x in scores.personLoc.itervalues()], 50,
       histtype='step')

  # Plot a model for the recall vs. speedup assuming resampling
  nWindows, maxHogTime, EstimateTime = PlottingUtils.ParseHogTiming(options)
  nPeople = len(scores.windowCount)
  fracWindows = np.arange(1.0, 40.0)
  nFound = np.array([sum([min(count/frac,1) for count in nonZeroCounts])
            for frac in fracWindows])
  meanRecall = np.divide(nFound, nPeople)
  nVariance = np.array([sum([GetVarianceOfOnePerson(count, frac)
                             for count in nonZeroCounts])
                        for frac in fracWindows])
  stdRecall = np.sqrt(np.divide(nVariance, nPeople*nPeople))
  speedup = [maxHogTime / EstimateTime(nWindows / frac) for
             frac in fracWindows]
  figure()
  plot(speedup, meanRecall, lw=2, hold=True)
  fill_between(speedup, meanRecall - stdRecall, meanRecall + stdRecall,
               alpha=0.5)
  
  show()
