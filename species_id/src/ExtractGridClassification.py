#! /usr/bin/python
'''Script that extracts the fish classification for the entire possible grid of bounding boxes. Stores the results in a bag file.'''
usage='ExtractGridClassification.py [options]'

import roslib; roslib.load_manifest('species_id')
import rospy
import rosbag
import numpy as np
from optparse import OptionParser
import os
import os.path
import random
import re
import csv
import subprocess
import string
import time
import gc
from reefbot_msgs.msg import ImageRegion
from sensor_msgs.msg import RegionOfInterest
from objdetect_msgs.msg import DetectObjectGrid
from species_id.srv import SpeciesIDGrid
from species_id.msg import SpeciesIDScoring
from ParameterSetter import ParameterSetter
import copy
import cv2
from cv_blobs import Blob
import cv_bridge
from multiprocessing import Process

def ParseFishLabels(filename, videoRegexp, blobIdRegexp, blobRegexp):
  '''Parses the input csv so that each entry becomes a tuple of (blobFile, blobId, videoId, label).'''
  retVal = []

  f = open(filename)
  try:
    reader = csv.reader(f)
    for line in reader:
      imageFile = line[0]
      label = line[1]
      tup = (blobRegexp.search(imageFile).groups()[0],
             int(blobIdRegexp.search(imageFile).groups()[0]),
             videoRegexp.search(imageFile).groups()[0],
             label)
      retVal.append(tup)

  finally:
    f.close()

  return retVal

def ParseNegList(filename, blobRegexp, videoRegexp):
  '''Parses a list of negative examples, on per line, so that each entry is a tuple of (blobFile, videoId).'''
  retVal = []

  f = open(filename)
  try:
    for line in f:
      imageFile = line.strip()
      tup = (blobRegexp.search(imageFile).groups()[0],
             videoRegexp.search(imageFile).groups()[0])
      retVal.append(tup)

  finally:
    f.close()

  return retVal

def WriteNegativeExamples(labelFile, negEntries, nDesired, blobDir):
  entriesAdded = 0
  mixEntries = copy.deepcopy(negEntries)
  random.shuffle(mixEntries)
  for blobFile, videoId in mixEntries:
    if entriesAdded >= nDesired:
      break

    # Open the blob file and count the number of lines because the
    # number of blobs in there will be nLines - 1
    curBlobId = 0
    blobStream = open(os.path.join(blobDir, blobFile))
    try:
      garb = blobStream.readline()
      for line in blobStream:
        labelFile.write('%s,%i,1\n' % (blobFile, curBlobId))
        curBlobId += 1
    finally:
      blobStream.close()

    entriesAdded += curBlobId

  rospy.loginfo('Using %s negative examples' % entriesAdded)

def GetRandomRunName():
  retval = ''.join(random.choice(string.letters) for i in xrange(10))
  rospy.loginfo('Name of this run is ' + retval)
  return retval

def BuildFishIndex(labelFilename, indexFilename, blobDir, paramList):
  retval = 1
  try:
    procSequence =  ['bin/BuildFishIndex', '--input', labelFilename,
                     '--output', indexFilename,
                     '--blob_prefix', blobDir + '/'] + \
                     ['--%s=%s' % (x[0], x[1]) for x in paramList]
    rospy.loginfo("Running %s", procSequence)
    retval = subprocess.call(procSequence)
  finally:
    if retval <> 0:
      raise Exception("Could not build the fish index. Return code: %i\n %s" %
                      (retval, ' '.join(procSequence)))

def StartSpeciesIDNode(outputDir, namespace, outBag, bagTime, paramList,
                       indexFilename):
  rospy.loginfo('Running the SpeciesIDNode')
  nodeName = '%s_species_id_node' % namespace
  serviceName = '%s/detect_object_grid_service' % nodeName
  params = copy.deepcopy(paramList)
  params.append(('index_file', indexFilename))
  parameterHandler = ParameterSetter(nodeName, params, outBag, 'parameters',
                                     bagTime)

  vuEnv = os.environ
  #vuEnv['CPUPROFILE'] = '/home/mdesnoye/tmp/fish_classifier.prof'

  proc = subprocess.Popen(['bin/SpeciesIDRosNode',
                           '__name:=' + nodeName,
                           '__log:=%s/%s_log.out' % (outputDir, nodeName),
                           'species_id_grid_service:=%s' % serviceName],
                          env=vuEnv)

  rospy.loginfo('Waiting for the SpeciesIDNode to startup')
  rospy.wait_for_service(serviceName, timeout=6000)
  rospy.loginfo('SpeciesIDNode is running')

  serviceProxy = rospy.ServiceProxy(serviceName, SpeciesIDGrid)

  return (proc, parameterHandler, serviceProxy)

def RunOneExtraction(fishEntries,
                     negEntries,
                     blobDir,
                     experimentDir,
                     outputPrefix,
                     videoId=None,
                     shapeDictFilename=None,
                     useSurfDescriptor=False,
                     useSiftDescriptor=False,
                     useOpponentSurf=False,
                     useCInvariantSurf=False,
                     saveIndex=False,
                     hessThresh=400.0,
                     numNegBlobs=5.0,
                     options=None):

  # Split up the training and test entries
  if videoId is not None:
    trainEntries = filter(lambda x: x[2] <> videoId, fishEntries)
    trainNegEntries = filter(lambda x: x[1] <> videoId, negEntries)
    testEntries = filter(lambda x: x[2] == videoId, fishEntries)
    testBlobFiles = sorted(set([x[0] for x in testEntries]))
    rospy.loginfo('Testing with entries of video id ' + videoId)
  else:
    rospy.logfatal('Must have a video id. We are done')
    return
  

  runName = GetRandomRunName()

  # Now build up a list of training and negative entries entries to be
  # used by the BuildFishIndex program.
  labelFilename = os.path.join(experimentDir, 'train_%s.label' % runName)
  labelFile = open(labelFilename, 'w')
  try:
    for blobFile, blobId, trainVideoId, label in trainEntries:
      labelFile.write('%s,%i,%s\n' % (blobFile, blobId, label))
    WriteNegativeExamples(labelFile, trainNegEntries,
                          numNegBlobs*len(trainEntries),
                          blobDir)
  finally:
    labelFile.close()

  # Setup the parameters used by the routines
  paramList = [
    ('color_dict_filename', ''),
    ('color_converter', ''),
    ('color_frac', 0),
    ('shape_dict_filename', shapeDictFilename),
    ('surf_detector', True),
    ('surf_hessian_threshold', hessThresh),
    ('surf_octaves', 3),
    ('surf_octave_layers', 4),
    ('surf_extended', False),
    ('surf_descriptor', useSurfDescriptor),
    ('sift_descriptor', useSiftDescriptor),
    ('shape_weight', 1.0),
    ('min_shape_val', 0.01),
    ('min_color_val', 0.01),
    ('min_score', 0.005),
    ('opponent_color_surf', useOpponentSurf),
    ('cinvariant_color_surf', useCInvariantSurf),
    ('use_bounding_box', True),
    ('bounding_box_expansion', 1.0),
    ('geometric_rerank', False),
    ('geo_rerank_inlier_thresh', 3.0)]
  rospy.loginfo('Using parameters: ' + str(paramList))

  indexFilename = os.path.join(experimentDir, 'train_%s.index' % runName)

  # Build the search index for the fish
  BuildFishIndex(labelFilename, indexFilename, blobDir, paramList)

  # Open up the bag which we will output stuff to
  resultFile = '%s_%s.bag' % (outputPrefix, videoId)
  outBag = rosbag.Bag(os.path.join(experimentDir, resultFile), 'w', 'bz2')

  # Start a SpeciesIDNode to serve the requests and grab the service
  # hook to it
  speciesIdProc, speciesIdParams, ClassifyGrid = StartSpeciesIDNode(
    experimentDir, runName, outBag, rospy.Time.from_sec(0.0), paramList,
    indexFilename)
  try:
    blobSerializer = Blob.BlobSerializer()
    cvBridge = cv_bridge.CvBridge()
    npBridge = cv_bridge.NumpyBridge()
    curFrameNum = 0
    for blobFile in testBlobFiles:
      # The result message to store
      resultMsg = SpeciesIDScoring()
      
      # Read the blob file
      blobStream = open(os.path.join(blobDir, blobFile), 'r')
      try:
        blobs, imageFile = blobSerializer.Deserialize(blobStream, blobDir)
      finally:
        blobStream.close()
      
      rospy.loginfo('Processing %s' % imageFile)

      # Fill out the ground truth in the result message
      curEntries = filter(lambda x: x[0] == blobFile, testEntries)
      curEntries = sorted(curEntries, key=lambda x: x[1])
      curBlobId = 0
      for blob in blobs:
        curEntry = [x for x in curEntries if x[1] == curBlobId]
        if len(curEntry) == 1 and int(curEntry[0][3]) > 3:
          # The human label says it's a fish so add the label to the results
          resultMsg.labels.append(int(curEntries[curBlobId][3]))
          bbox = RegionOfInterest()
          bbox.x_offset = blob.minX
          bbox.y_offset = blob.minY
          bbox.height = blob.maxY - blob.minY
          bbox.width = blob.maxX - blob.minX
          resultMsg.regions.append(bbox)

        curBlobId += 1

      # Open up the image and package it into a message
      cvImage = cv2.imread(imageFile)
      imgMsg = cvBridge.cv_to_imgmsg(cvImage, 'bgr8')
      imgMsg.header.stamp = rospy.Time.from_sec(curFrameNum)
      imgMsg.header.seq = curFrameNum

      # Build up the request
      request = DetectObjectGrid()
      request.header = imgMsg.header
      request.image = imgMsg
      request.grid.minX = 0
      request.grid.minY = 0
      request.grid.minH = options.min_region_height
      request.grid.minW = options.min_region_width
      request.grid.strideX = options.win_stride
      request.grid.strideY = options.win_stride
      request.grid.strideH = options.scale_stride
      request.grid.strideW = options.scale_stride
      request.grid.fixAspect = False
      request.mask.encoding = "8U"

      # Process this image
      response = ClassifyGrid(request)

      # Build up the result message to store
      resultMsg.image = imageFile
      resultMsg.grid = response.grid
      resultMsg.confidence = response.confidence
      resultMsg.top_label = response.top_label
      resultMsg.not_fish_confidence = response.not_fish_confidence
      resultMsg.processing_time = response.processing_time

      # Record the result in the bag file
      outBag.write('results', resultMsg, request.header.stamp)
      outBag.flush()

      curFrameNum += 1
    

  finally:
    outBag.close()
    speciesIdParams.RemoveParameters()
    ClassifyGrid.close()
    speciesIdProc.send_signal(2)
    tries = 0
    while speciesIdProc.returncode == None and tries < 10:
      speciesIdProc.poll()
      tries = tries+1
      time.sleep(1)
    if speciesIdProc.returncode == None:
      speciesIdProc.kill()

    if not options.save_index:
      os.remove(indexFilename)
      
    gc.collect()
  

if __name__ == '__main__':
  # All of the command line flags
  parser = OptionParser(usage=usage)

  parser.add_option('--blob_dir', dest='blob_dir',
                    help='Directory where the blob files reside. Must include positive and negative blobs.',
                    default=None)
  parser.add_option('--experiment_dir', dest='experiment_dir',
                    help='Directory to put the files used in the experiment and the results',
                    default=None)
  parser.add_option('--output_prefix', default='',
                    help='The prefix for the output bag filename')
  parser.add_option('--image_list', dest='image_list',
                    help='CSV file of <image_filename>,<label> listing all the entries for known fish')
  parser.add_option('--neg_blob_list', default=None,
                    help='Text file with a filename on each line. One per blob file that contains negative blobs.')
  parser.add_option('--num_neg_blobs', type='float', default=5.0,
                    help='Number of negative examples to add to the index as a multiple of positive examples')
  parser.add_option('--video_regexp', dest='video_regexp',
                    help='Regular expression used to extract the video id from the filename',
                    default='(([0-9][0-9]-){3})')
  parser.add_option('--blobid_regexp', dest='blobid_regexp',
                    help='Regular expression used to extract the blob id from the filename',
                    default='blob\.([0-9]+)\.')
  parser.add_option('--blob_regexp', dest='blob_regexp',
                    help='Regular expression used to extract the blob filename from the image filename',
                    default='/*(.+\.blob)')
  parser.add_option('--shape_dict_filename',
                    help='Filename of the shape dictionary',
                    default='/data/mdesnoye/fish/experiments/extraction081011/20110102/osurf.dict')
  parser.add_option('--use_surf_descriptor', action='store_true',
                    default=False, help='Use SURF descriptors?')
  parser.add_option('--use_sift_descriptor', action='store_true',
                    default=False, help='Use SIFT descriptors?')
  parser.add_option('--use_opponent_surf', action='store_true',
                    default=False, help='Use Opoonent color SURF descriptors?')
  parser.add_option('--use_cinvariant_surf', action='store_true',
                    default=False, help='Use C-Invariant SURF descriptors?')
  parser.add_option('--video_ids', default=None,
                    help='Python code specifying which video ids to use')
  parser.add_option('--save_index', action='store_true', default=False,
                    help="Save the index that was built")
  parser.add_option('--hess_thresh', default=300, type="float",
                    help='Hessian threshold used to find interest points.')
  parser.add_option('--random_seed', default=495198, type='int')

  # Sampling parameters
  parser.add_option('--min_region_width', type='int', default=32,
                    help='Width of the minimum region to evaluation in pixels')
  parser.add_option('--min_region_height', type='int', default=32,
                    help='Height of the minimum region to evaluation in pixels')
  parser.add_option('--win_stride', type='int', default=16,
                    help='When sampling, the stride in pixels for identifying regions')
  parser.add_option('--scale_stride', type='float', default=1.50,
                    help='When sampling, the scaling factor between levels')

  parser.add_option('--do_testing', action='store_true', default=False,
                    help='Set to be in a testing mode')

  (options, args) = parser.parse_args()

  random.seed(options.random_seed)

  rospy.init_node('ExtractGridClassification', anonymous=True)

  fishEntries = ParseFishLabels(options.image_list,
                                re.compile(options.video_regexp),
                                re.compile(options.blobid_regexp),
                                re.compile(options.blob_regexp))
  negEntries = ParseNegList(options.neg_blob_list,
                            re.compile(options.blob_regexp),
                            re.compile(options.video_regexp))

  # Get the list of unique video ids
  if options.video_ids is None:
    videoIds = set([x[2] for x in fishEntries])
  else:
    videoIds = eval(options.video_ids)

  for videoId in videoIds:
    args = (fishEntries,
            negEntries,
            options.blob_dir,
            options.experiment_dir,
            options.output_prefix)
    kwargs = {'videoId': videoId,
              'shapeDictFilename': options.shape_dict_filename,
              'useSurfDescriptor': options.use_surf_descriptor,
              'useSiftDescriptor': options.use_sift_descriptor,
              'useOpponentSurf': options.use_opponent_surf,
              'useCInvariantSurf': options.use_cinvariant_surf,
              'saveIndex': options.save_index,
              'hessThresh': options.hess_thresh,
              'numNegBlobs': options.num_neg_blobs,
              'options': options}
    if options.do_testing:
      RunOneExtraction(*args, **kwargs)

    else:
      p = Process(target=RunOneExtraction,
                  args=args, kwargs=kwargs)
      p.start()
      p.join()
