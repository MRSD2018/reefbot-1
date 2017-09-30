import roslib; roslib.load_manifest('hima_experiment')
import rospy
import os
import os.path
import re
import numpy as np
import scipy.interpolate

def AddCommandLineOptions(parser):
  '''Add command line options that are needed for routines in here.'''
  parser.add_option('--annotation_regex',
                    default='annotations_([\w]+)\.txt',
                    help='Regular expression to match an annotations file and to extract the dataset from it')
  parser.add_option('--root_results_dir',
                    default='/data/mdesnoye/pedestrian/vu_estimation/eth/',
                    help='Root directory of the results. We will search all subdirectories of this one')
  parser.add_option('--dataset_regex', default='_([0-9A-Za-z]+)\.bag',
                    help='Regexp to extract the dataset name from the filename')
  parser.add_option('--estimator_regex',
                    default='vu_([^\s]+)_[0-9A-Za-z]+\.bag',
                    help='Regexp to extract the estimator from the bag filename')
  parser.add_option('--bag_regex', default='vu_.*\.bag',
                    help='Regexp to select which bags to process')
  parser.add_option('--vu_timing_regex',
                    default='%s_timing\.txt',
                    help='Regex to find a timing file that specifies the time to compute a frame given a value of the threshold. Takes one parameter, which is the vu type')
  parser.add_option('--valid_dataset', default='(?!test).*',
                    help='Regex to match a valid dataset')
  parser.add_option('--valid_vutype', default='.*',
                    help='Regex to match a valid visual utility type')
  parser.add_option('--hog_timing_file',
                    default='/data/mdesnoye/pedestrian/vu_estimation/eth/hog/hog_timing.txt',
                    help='File that specifies the timing for the hog')
  parser.add_option('--hog_name', help='Directory containing the hog bags',
                    default='HOGDetector')
  return parser

def ParseAnnotationFiles(options, annotationDir=None):
  '''Returns a dataset -> annotationFile map.'''
  annotationRe = re.compile(options.annotation_regex)
  retval = {}
  if annotationDir is None:
    annotationDir = options.root_results_dir
  for filename in os.listdir(annotationDir):
    match = annotationRe.match(filename)
    if match:
      database = match.groups()[0]
      retval[database] = os.path.join(annotationDir, filename)

  return retval

def FindBagsInDir(root, files, options):
  retval = {}
  bagRe = re.compile(options.bag_regex)
  estimatorRe = re.compile(options.estimator_regex)
  datasetRe = re.compile(options.dataset_regex)
  for filename in files:
    if bagRe.match(filename):
      estimatorMatch = estimatorRe.search(filename)
      if estimatorMatch is None:
        rospy.logwarn('Could not find the estimator name in: %s' % (filename))
        continue
      
      estimator = estimatorMatch.groups()[0]
      dataset = datasetRe.search(filename).groups()[0]
      retval[(estimator, dataset)] = os.path.join(root, filename)

  return retval
  

def FindVUBags(options, useRoot=False):
  '''Finds the vu bags to process.

  returns ((VUType, Dataset)->filename map, datasets, vuTypes).'''
  bags = {}
  for root, dirs, files in os.walk(options.root_results_dir,
                                   followlinks=True):
    if os.path.samefile(root, options.root_results_dir):
      if useRoot:
        bags.update(FindBagsInDir(root, files, options))
      continue
    if not useRoot:
      bags.update(FindBagsInDir(root, files, options))

  
  # Figure  out the datasets and vu types
  validVuRegex = re.compile(options.valid_vutype)
  validDatasetRegex = re.compile(options.valid_dataset)
  datasets  = set([dataset for x, dataset in bags.iterkeys() if
                   validDatasetRegex.match(dataset)])
  vuTypes = set([vuType for vuType, x in bags.iterkeys()
                 if (vuType <> options.hog_name and
                     validVuRegex.match(vuType))])

  return (bags, datasets, vuTypes)

def ParseVuTimingData(filename):
  '''Parses a csv file where each line is threshold,cpuTime.
  Return a tuple (thresholds, runtime)
  '''
  thresholds = []
  runtime = []
  for line in open(filename):
    splitLine = line.strip().split(',')
    thresholds.append(float(splitLine[0]))
    runtime.append(float(splitLine[1]))
  return (np.array(thresholds), np.array(runtime))

def FindTimingData(vutypes, options):
  '''Finds the timing data for each VUType if they are available.
  Returns VUType->(thresholds, runtime) map'''
  retval = {}
  for vutype in vutypes:
    timingRe = re.compile(options.vu_timing_regex % vutype)
    retval[vutype] = None

    for root, dirs, files in os.walk(options.root_results_dir):
      for filename in files:
        if timingRe.match(filename):
          retval[vutype] = ParseVuTimingData(os.path.join(root, filename))

  return retval

def ParseHogTiming(options):
  '''Reads a hog timing file and returns (whole frame time, interpObject)'''
  nWindows = []
  times = []
  for line in open(options.hog_timing_file):
    splitLine = line.strip().split(',')
    nWindows.append(int(splitLine[0]))
    times.append(float(splitLine[1]))

  interpObj = scipy.interpolate.interp1d(np.array(nWindows),
                                         np.array(times),
                                         kind='cubic')

  return (max(nWindows), times[-1], interpObj)
