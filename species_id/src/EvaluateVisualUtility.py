#!/usr/bin/python
'''Evaluates the visual utility for a set of images in HIMA format

This script will output a series of bag files. Each bag file will have the following topics:

/parameters - List of Parameter messages one for each parameter of the run
/results - Sequence of VisualUtilityEstimation messages specifying the scores
'''
usage='EvaluateVisualUtility.py [options]'

import roslib; roslib.load_manifest('species_id')
import rospy
import rosbag
import gc
from optparse import OptionParser
from ParameterSetter import ParameterSetter
from objdetect_msgs.srv import DetectObjectGridService
from sensor_msgs.msg import MatND
from objdetect_msgs.msg import DetectObjectGrid
from species_id.msg import VisualUtilityEstimation
import FishDataLoader
import cv_bridge
import cv2
import os
import os.path
import subprocess
import time
import random
import string
import numpy as np
import re
from multiprocessing import Process

def StartVisualUtilityEstimator(outputDir, namespace, outBag, bagTime, options):
  rospy.loginfo('Running the Visual Utility Estimator')

  nodeName = '%s_vu_estimator' % namespace
  serviceName = '%s/detect_object_grid_service' % nodeName
  parameterList = [('affine_max_iterations', options.affine_max_iterations),
                   ('min_affine_precision', options.min_affine_precision),
                   ('affine_scaling_factor', options.affine_scaling_factor),
                   ('vu_estimator', options.vu_estimator),
                   ('vu_estimator_wrapper', options.vu_estimator_wrapper),
                   ('pareto_thresh', options.pareto_thresh),
                   ('dist_decay', options.dist_decay),
                   ('opening_size', options.opening_size),
                   ('laplacian_size', options.laplacian_size),
                   ('hist_dist_type', options.hist_dist_type),
                   ('estimator_scales', options.estimator_scales),
                   ('hog_model_file', options.hog_model_file),
                   ('hog_do_people', options.hog_do_people),
                   ('hog_do_cache', options.hog_do_cache),
                   ('win_stride', options.win_stride),
                   ('img_scaling', options.img_scaling),
                   ('cascade_model_file', options.cascade_model_file)]
  parameters = ParameterSetter(nodeName,
                               parameterList,
                               outBag, 'parameters', bagTime)

  vuEnv = os.environ
  #vuEnv['CPUPROFILE'] = '/home/mdesnoye/tmp/vu_saliency.prof'

  proc = subprocess.Popen(['../visual_utility/bin/VisualUtilityEstimatorNode',
                           '__name:=' + nodeName,
                           '__log:=%s/%s_log.out' % (outputDir, nodeName),
                           'vu_objdetect_grid_service:=%s' % serviceName],
                          env=vuEnv)

  rospy.loginfo('Waiting for the visual utility estimator to startup')
  rospy.wait_for_service(serviceName,
                         timeout=6000)
  rospy.loginfo('Visual utility filter is running')
  serviceProxy = rospy.ServiceProxy(serviceName, DetectObjectGridService)
  
  return (proc, parameters, serviceProxy)
  

def GetRandomRunName():
  retval = ''.join(random.choice(string.letters) for i in xrange(10))
  rospy.loginfo('Name of this run is ' + retval)
  return retval

def SendFramesToVisualUtilityNode(movieFn, lastFrameSent, desiredFrameNum,
                                  vuService, fps, movieSubsetRate):
  cvBridge = cv_bridge.CvBridge()
  movieStream = cv2.VideoCapture(movieFn)

  curFrameNum = 0
  rospy.loginfo('Last frame sent: %i. Desired frame %i' %(lastFrameSent,
                                                          desiredFrameNum))
  while curFrameNum < desiredFrameNum and movieStream.grab():
    if curFrameNum > lastFrameSent and (curFrameNum % movieSubsetRate == 0):
      gotFrame, cvImage = movieStream.retrieve()
      if not gotFrame:
        rospy.logerr('Error retrieving frame %i from %s/%s.mp4' %
                     (curFrameNum, options.movie_dir, frame.movieId))
        break
      request = DetectObjectGrid()
      request.image = cvBridge.cv_to_imgmsg(cvImage, 'bgr8')
      request.header.seq = curFrameNum
      request.header.stamp = rospy.Time.from_sec(curFrameNum /
                                                 fps)
      request.image.header.seq = curFrameNum
      request.image.header.stamp = request.header.stamp
      request.grid.minX = 0
      request.grid.minY = 0
      request.grid.minH = 100
      request.grid.minW = 100
      request.grid.strideX = 10000
      request.grid.strideY = 10000
      request.grid.strideH = 10000
      request.grid.strideW = 10000
      
      response = vuService(request)
          
    curFrameNum += 1

  movieStream.release()
  if curFrameNum <> desiredFrameNum:
    rospy.logerr('We had a problem getting the frames in order for frame: %s' % frame.imgFile)
    return 1
  
  return 0

def CalculateVisualUtilityForFrameList(frames, outputDir, resultFile,
                                       options):
  runName = GetRandomRunName()
  cvBridge = cv_bridge.CvBridge()
  npBridge = cv_bridge.NumpyBridge()

  # Open up the bag which we will output stuff to
  outBag = rosbag.Bag(os.path.join(outputDir, resultFile), 'w', 'bz2')

  (vuProc, vuParams, vuService) = StartVisualUtilityEstimator(
    outputDir, runName, outBag, rospy.Time.from_sec(0.0), options)
  regions = []
  imageShape = None
  movieFn = None
  curMovieId = None
  try:
    curFrameNum = -1
    for frame in frames:      
      rospy.loginfo('Processing %s' % frame.imgFile)

      # Open up the image
      if options.use_movie:

        # Open the movie if necessary
        if curMovieId <> frame.movieId:
          movieFn = os.path.join(options.movie_dir, '%s.mp4' % frame.movieId)
          curFrameNum = -1
          curMovieId = frame.movieId

        # Advance through the movie, sending frames to the estimator
        # (although don't ask for many of the boxes because we're
        # throwing them out
        p = Process(target=SendFramesToVisualUtilityNode,
                    args=(movieFn, curFrameNum, frame.frameNum,
                          vuService, options.fps, options.movie_subset_rate))
        p.start()
        p.join()
        if p.exitcode <> 0:
          raise IOError('Could not advance the movie')
            
      if not os.path.exists(frame.imgFile):
        rospy.logerr('%s does not exists. Skipping' % frame.imgFile)
        continue
      cvImage = cv2.imread(frame.imgFile)
      curFrameNum = frame.frameNum
        
      image = cvBridge.cv_to_imgmsg(cvImage, 'bgr8')
      image.header.stamp = rospy.Time.from_sec(curFrameNum / options.fps)
      image.header.seq = curFrameNum

      # Calculate the visual utility for each region
      request = DetectObjectGrid()
      request.header = image.header
      request.image = image
      request.grid.minX = 0
      request.grid.minY = 0
      request.grid.minH = options.min_region_height
      request.grid.minW = options.min_region_width
      request.grid.strideX = options.win_stride
      request.grid.strideY = options.win_stride
      request.grid.strideH = options.scale_stride
      request.grid.strideW = options.scale_stride
      request.grid.fixAspect = False
      response = vuService(request)

      if (len(response.scores.scores.data) == 0):
        rospy.logerr('Did not receive scores for: %s' % frame.imgFile)

      # Build up the result message to store
      resultMsg = VisualUtilityEstimation()
      resultMsg.header = request.header
      resultMsg.image = frame.imgFile
      resultMsg.scores = response.scores

      # Record the result in the bag file
      outBag.write('results', resultMsg, resultMsg.header.stamp)
      outBag.flush()

      del request
      del response
      del image
      gc.collect()

  finally:
    outBag.close()
    vuParams.RemoveParameters()
    vuService.close()
    vuProc.send_signal(2)
    tries = 0
    while vuProc.returncode == None and tries < 10:
      vuProc.poll()
      tries = tries+1
      time.sleep(1)
    if vuProc.returncode == None:
      vuProc.kill()
    gc.collect()

if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('--blob_dir',
                    help='Directory containing the blob files',
                    default=None)
  parser.add_option('--movie_dir',
                    help='Directory containing the video files',
                    default=None)
  parser.add_option('--parsing_regex',
                    default='((([0-9][0-9]-){3})([0-9]+))\.',
                    help='Regex to extract the frame id and video id from filenames')
  parser.add_option('--output_dir',
                    help='Directory to output the results to')
  parser.add_option('--output_name', default=None,
                    help='Unique name for the outputs. Defaults to the estimator class')
  parser.add_option('--use_movie', action='store_true', default=False,
                    help='Set if the movie is needed to be processed in order to extract the visual utility scores. This will be true for any estimator that contains state across frames')
  parser.add_option('--fps', type='float', default=30.0,
                    help='Frame rate of the video')
  parser.add_option('--movie_subset_rate', default=1, type='int',
                    help='Subsampling rate for the movie')

  # Sampling parameters
  parser.add_option('--min_region_width', type='int', default=32,
                    help='Width of the minimum region to evaluation in pixels')
  parser.add_option('--min_region_height', type='int', default=32,
                    help='Height of the minimum region to evaluation in pixels')
  parser.add_option('--win_stride', type='int', default=16,
                    help='When sampling, the stride in pixels for identifying regions')
  parser.add_option('--scale_stride', type='float', default=1.50,
                    help='When sampling, the scaling factor between levels')

  # For the transform estimator
  parser.add_option('--transform_estimator',
                    help='String specifying the transform estimator to use.',
                    default='AffineTransformEstimator')
  parser.add_option('--affine_max_iterations', type='int',
                    help='Maximum number of iterations when determining the affine transform',
                    default=100)
  parser.add_option('--min_affine_precision', type='float',
                    help='Minimum precision of the affine transform',
                    default=1e-7)
  parser.add_option('--affine_scaling_factor', type='float',
                    help='Scaling factor to speed up the affine estimation',
                    default=2.0)

  # For the visual utility esimator
  parser.add_option('--vu_estimator',
                    help='String specifying the visual utility estimator to use',
                    default='LABMotionVUEstimator')
  parser.add_option('--vu_estimator_wrapper',
                    help='String specifying the visual utility estimator wrapper to use',
                    default='')
  parser.add_option('--pareto_thresh', type='float',
                    help='Fraction of pixels to keep assuming pareto distribution of moving pixels',
                    default=0.03)
  parser.add_option('--dist_decay', type='float',
                    help='Sigmoid decay around the threshold',
                    default=2.0)
  parser.add_option('--opening_size', type='int',
                    help='Size of region for morphological opening. Should be odd',
                    default=3)
  parser.add_option('--laplacian_size', type='int',
                    help='Size of the laplacian aperture in pixels.',
                    default=3)
  parser.add_option('--hist_dist_type',
                    help='Type of distance metric to use to compare histograms',
                    default='chisq')
  parser.add_option('--estimator_scales',
                    help='Comma separated float specifying a set of scales to use for the estimator',
                    default='1.0')
  parser.add_option('--hog_model_file',
                    help="File to load into the hog detector",
                    default="")
  parser.add_option('--hog_do_people', action='store_true',
                    dest='hog_do_people', default=True)
  parser.add_option('--no_hog_do_people', action='store_false',
                    dest='hog_do_people')
  parser.add_option('--hog_do_cache', action='store_true',
                    dest='hog_do_cache', default=False)
  parser.add_option('--cascade_model_file',
                    help="File to load into the cascade detector",
                    default="")
  parser.add_option('--img_scaling', type='float', default=0.5,
                    help="How much to scale the image if using ScaledDetectorWrapper")
  parser.add_option('--do_testing', action='store_true', default=False,
                    help='Set to be in a testing mode')
                    

  (options, args) = parser.parse_args()

  rospy.init_node('EvaluateVisualUtilityFish', anonymous=True)

  frameList = FishDataLoader.LoadFrameInfoFromBlobs(
    re.compile(options.parsing_regex),
    options.blob_dir)

  outputName = options.output_name
  if outputName is None:
    outputName = options.vu_estimator

  movieList = set([x.movieId for x in frameList])

  # Compute each of the movies seperates so that the data doesn't get too big
  for movieId in movieList:
    outputFn = 'vu_%s_%s.bag' % (outputName, movieId)

    curFrames = [x for x in frameList if x.movieId == movieId]

    # We run as a seperate process so that when it finishes, the
    # memory can be cleaned up properly. Otherwise, we run out.
    if options.do_testing:
      CalculateVisualUtilityForFrameList(curFrames,
                                         options.output_dir,
                                         outputFn,
                                         options)
    else:
      p = Process(target=CalculateVisualUtilityForFrameList,
                  args=(curFrames,
                        options.output_dir,
                        outputFn,
                        options))
      p.start()
      p.join()
