#!/usr/bin/python
'''Calculates the mean and standard deviation of the processing times in a bag. Also determines the fraction of ground truth positive examples per image.
'''
usage='CalculateProcessingTime.py <bag0> <bag1> ...' 

import roslib; roslib.load_manifest('hima_experiment')
import rospy
import rosbag
import numpy as np
import math
from optparse import OptionParser
from pylab import *

def ShowStats(data, dataType):
  data = np.array(data)
  curMean = np.mean(data)
  stdDev = np.std(data)

  # Filter out the outliers
  data = data[np.abs(data - curMean) < 3.0*stdDev]
  stdDev = np.std(data)

  rospy.loginfo('Statistics for ' + dataType)
  rospy.loginfo('Mean: %f s'  % np.mean(data))
  rospy.loginfo('Standard Deviation: %f' % stdDev)
  rospy.loginfo('Standard Error: %f' % (stdDev / math.sqrt(len(data))))

  figure()
  hist(data, 50)
  xlabel(dataType)

if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('--overlap', type='float', default=0.5,
                    help='Fraction overlap to consider a window a ground truth hit.')


  (options, args) = parser.parse_args()

  processingTimes = []
  fracPositive = []

  # Load up the processing times
  for bagFile in args:
    rospy.loginfo('Reading bag %s' % bagFile)
    bag = rosbag.Bag(bagFile)
    try:
      for topic, msg, t in bag.read_messages(topics=['results']):
        if topic == 'results':
          processingTimes.append(msg.processing_time.data.to_sec())
          fracPositive.append(
            np.sum(np.array(msg.overlaps) > options.overlap) /
            float(len(msg.overlaps)))
    finally:
      bag.close()

  ShowStats(processingTimes, "Processing Time")
  
  ShowStats(fracPositive, "Fraction of Positive Windows")

  show()
