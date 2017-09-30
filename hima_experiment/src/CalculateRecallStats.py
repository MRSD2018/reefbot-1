import roslib; roslib.load_manifest('hima_experiment')
import rospy
import rosbag
import numpy as np
import PlottingUtils
import scipy.interpolate
import re
import HimaDataLoader
from optparse import OptionParser
import VUAccuracy

if __name__ == '__main__':
  parser = OptionParser()

  parser = PlottingUtils.AddCommandLineOptions(parser)

  parser.add_option('--hog_thresh', type='float',
                    help='Minimum HOG value required to label a hit',
                    default=0.0)
  parser.add_option('--frame_subset_rate', type='int', default=1,
                    help='In order to run quicker, you can only run the detector on a subset of the frames. This specifies how often to run the detector. So, for example if it is 5, it will run the detector once every 5 frames.')
  parser.add_option('--imageid_regex', default='([0-9]+)\.((png)|(jpg)|(bmp))',
                    help='Regex to extract the image id from a filename')
  parser.add_option('--hog_name', help='Directory containing the hog bags',
                    default='HOGCachedDetector')
  parser.add_option('--min_overlap', type='float',
                    help='Minimum fraction overlap for a target to be correct',
                    default=0.5)
  parser.add_option('--cache_dir',
                    default='/data/mdesnoye/pedestrian/vu_estimation/eth/cache',
                    help='Directory to use to cache subcomponents in this data so that it can be computed faster')
  
  (options, args) = parser.parse_args()

  hogProcessingTime, hogTiming = PlottingUtils.ParseHogTiming(options)

  timingData = PlottingUtils.FindTimingData(vuTypes, options)

  annotationsMap = PlottingUtils.ParseAnnotationFiles(options)
  
  bags, datasets, vuTypes = PlottingUtils.FindVUBags(options)

  hogScore = VUAccuracy.CreateVUScore(
    [bags[(options.hog_name, x)] for x in datasets],
    [annotationsMap[x] for x in datasets],
    options.frame_subset_rate,
    False, # doNMS
    options.hog_thresh,
    options.min_overlap,
    cacheDir=options.cache_dir,
    imageIdRegex=options.imageid_regex)

  for vuType in vuTypes:
    vuScores = VUAccuracy.CreateVUScore(
      [bags[(vuType, x)] for x in datasets],
      [annotationsMap[x] for x in datasets],
      options.frame_subset_rate,
      thresholdTimes=timingData[vuType],
      cacheDir=options.cache_dir,
      imageIdRegex=options.imageid_regex)
