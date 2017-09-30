#!/usr/bin/python
'''Runs a hima cascade experiment where there is a visual utility estimator
 and a high level hog algorithm. All the timing is done properly.

 This script will output a series of bag files. Each bag file will have the following topics:

/parameters - List of Parameter messages one for each parameter of the run
/results - Sequence of HimaCascadeResult messages specifying the scores
'''
usage='RunHimaCascade.py [options]'

import roslib; roslib.load_manifest('hima_experiment')
import rospy
import rosbag

from optparse import OptionParser
from ParameterSetter import ParameterSetter
from objdetect_msgs.srv import DetectObjectService
from sensor_msgs.msg import RegionOfInterest
from objdetect_msgs.msg import DetectObject
from hima_experiment.msg import HimaCascadeResult
import HimaDataLoader
import DetectionUtils
import cv_bridge
import cv2
import os
import os.path
import subprocess
import time
import random
import string
import numpy as np
from multiprocessing import Process

def GetRandomRunName():
  retval = ''.join(random.choice(string.letters) for i in xrange(10))
  rospy.loginfo('Name of this run is ' + retval)
  return retval

def GetVUEstimatorParameters(options):
  return [('affine_max_iterations', options.affine_max_iterations),
          ('min_affine_precision', options.min_affine_precision),
          ('affine_scaling_factor', options.affine_scaling_factor),
          ('vu_estimator', options.vu_estimator),
          ('vu_estimator_wrapper', options.vu_estimator_wrapper),
          ('pareto_thresh', options.pareto_thresh),
          ('dist_decay', options.dist_decay),
          ('laplacian_size', options.laplacian_size),
          ('hist_dist_type', options.hist_dist_type),
          ('estimator_scales', options.estimator_scales),
          ('hog_model_file', options.cascade_model_file),
          ('win_stride', options.win_stride),
          ('img_scaling', options.img_scaling),
          ('cascade_model_file', options.cascade_model_file)]

def GetHogDetectorParameters(options):
  return [('vu_estimator', 'HOGDetector'),
          ('hog_model_file', options.hog_model_file),
          ('hog_do_people', options.hog_do_people),
          ('hog_do_cache', options.hog_do_cache),
          ('win_stride', options.win_stride)]

def StartSubDetector(outputDir, namespace, nodeName, outBag, bagTime,
                     parameterList):
  rospy.loginfo('Running the subdetector %s' % nodeName)

  nodeId = '%s_%s' % (namespace, nodeName)
  serviceName = '%s/detect_object_service' % nodeId
  parameters = ParameterSetter(nodeId,
                               parameterList,
                               outBag, 'parameters_%s' % nodeName, bagTime)

  vuEnv = os.environ
  #vuEnv['CPUPROFILE'] = '/home/mdesnoye/tmp/%s_%s.prof' % (nodeName, namespace)
  #rospy.loginfo('Profiling the visual utility estimator to %s' %
  #              vuEnv['CPUPROFILE'])

  proc = subprocess.Popen(['../visual_utility/bin/VisualUtilityEstimatorNode',
                           '__name:=' + nodeId,
                           '__log:=%s/%s_log.out' % (outputDir, nodeId),
                           'vu_objdetect_service:=%s' % serviceName],
                          env=vuEnv)

  rospy.loginfo('Waiting for %s to startup' % nodeId)
  rospy.wait_for_service(serviceName,
                         timeout=6000)
  rospy.loginfo('%s is running' % nodeId)
  serviceProxy = rospy.ServiceProxy(serviceName, DetectObjectService)

  return (proc, parameters, serviceProxy)

def StopSubDetector(proc, parameters, serviceProxy):
  parameters.RemoveParameters()
  serviceProxy.close()
  proc.send_signal(2)
  tries = 0
  while proc.returncode == None and tries < 10:
    proc.poll()
    tries = tries+1
    time.sleep(1)
  if proc.returncode == None:
    proc.kill()

def RegionKey(region):
  '''Calculates a hash of a region message.'''
  return region.x_offset | (region.y_offset << 16) | \
         (region.height << 32) | (region.width << 48)
  

def RunHimaCascadeForOneDataset(frames, outputDir, resultFile, options):
  runName = GetRandomRunName()
  cvBridge = cv_bridge.CvBridge()

  # Open up the bag which we will output stuff to
  outBag = rosbag.Bag(os.path.join(outputDir, resultFile), 'w', 'bz2')

  # Start the visual utility filter
  (vuProc, vuParams, vuService) = StartSubDetector(
    outputDir,
    runName,
    'vu_filter',
    outBag,
    rospy.Time.from_sec(frames[0].timestamp),
    GetVUEstimatorParameters(options))
  try:

    # Start the hog detector
    (hogProc, hogParams, hogService) = StartSubDetector(
      outputDir,
      runName,
      'hog',
      outBag,
      rospy.Time.from_sec(frames[0].timestamp),
      GetHogDetectorParameters(options))
    try:
      regions = []
      imageShape = None
      curSeq = 0
      for frame in frames:
        curSeq += 1
      
        # See if we should skip sending the frame to the detector
        if (curSeq % options.frame_subset_rate <> 1 and
            options.frame_subset_rate <> 1):
          continue

        rospy.loginfo('Processing %s' % frame.imageFile)

        # Open up the image
        if not os.path.exists(frame.imageFile):
          rospy.logerr('%s does not exists. Skipping' % frame.imageFile)
          continue
        cvImage = cv2.imread(frame.imageFile)
        
        image = cvBridge.cv_to_imgmsg(cvImage,
                                      'bgr8')
        image.header.stamp = rospy.Time.from_sec(frame.timestamp)
        image.header.seq = curSeq

        if imageShape is None or imageShape <> cvImage.shape:
          regions = DetectionUtils.GenerateSamplingRegions(
            cvImage,
            options.min_region_width,
            options.min_region_height,
            options.win_stride,
            options.scale_stride)
          imageShape = cvImage.shape

        # Calculate the visual utility for each region
        vuRequest = DetectObject()
        vuRequest.image = image
        vuRequest.regions = regions
        vuResponse = vuService(vuRequest)

        if (len(vuResponse.detections.detections) <> len(regions)):
          rospy.logerr('Did not receive scores for all the regions in image: %s' % frame.imageFile)
          continue

        resultMsg = HimaCascadeResult()
        resultMsg.header.stamp = rospy.Time.from_sec(frame.timestamp)
        resultMsg.header.seq = curSeq - 1
        resultMsg.image = frame.imageFile
        resultMsg.processing_time.data = vuResponse.processing_time.data
        resultMsg.vu_processing_time.data = vuResponse.processing_time.data
        resultMsg.total_regions = [x.mask.roi for
                                   x in vuResponse.detections.detections]
        resultMsg.overlaps = [frame.GetMaxOverlap(x) for x in
                              resultMsg.total_regions]

        # Figure out which regions to pass to the hog detector
        vuScores = np.array([x.score for x in
                             vuResponse.detections.detections])
        vuThresh = options.vu_thresh
        if vuThresh is None:
          if np.max(vuScores) == np.min(vuScores):
            vuThresh = np.max(vuScores) + 1
          else:
            vuThresh = np.max(vuScores)
        resultMsg.pass_vu_regions = np.flatnonzero(vuScores >= vuThresh)

        if len(resultMsg.pass_vu_regions) > 0:
          # Pass the candidate regions to the hog detector
          hogRequest = DetectObject()
          hogRequest.image = image
          hogRequest.regions = [regions[i] for i in resultMsg.pass_vu_regions]
          hogResponse = hogService(hogRequest)

          # Build up the results
          resultMsg.processing_time.data += hogResponse.processing_time.data
          curScores = [x.score for
                       x in hogResponse.detections.detections]

          # The response will not necessarily keep the regions in the
          # same order, so we have to map back to their correct place
          # so that the scores ordering is correct.
          responseMap = {}
          for i in range(len(hogRequest.regions)):
            responseMap[RegionKey(hogResponse.detections.detections[i].mask.roi)] = i
          resultMsg.scores = [curScores[responseMap[RegionKey(x)]]
                              for x in hogRequest.regions]
          

        # Write the results to the bag
        outBag.write('results', resultMsg, resultMsg.header.stamp)
        outBag.flush()

    finally:
      StopSubDetector(hogProc, hogParams, hogService) 

  finally:
    outBag.close()
    StopSubDetector(vuProc, vuParams, vuService)
  

def ProcessOneDirectory(inputDir, options):
  outputName = options.output_name
  if outputName is None:
    outputName = options.vu_estimator
    
  dataset = os.path.basename(os.path.normpath(inputDir))

  frames = HimaDataLoader.LoadFrames(
      inputDir, options.left_image_dir,
      options.right_image_dir,
      options.annotations,
      imgFileString = options.image_file_string)

  outputFn = 'vu_cascade_%s_%s.bag' % (outputName, dataset)

  RunHimaCascadeForOneDataset(frames,
                              options.output_dir,
                              outputFn,
                              options)


if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('--input_dirs',
                    help='File specifying the input directories, one per line',
                    default=None)
  parser.add_option('--left_image_dir',
                    help='Relative path to the left camera image',
                    default='stereo_cameras/left')
  parser.add_option('--right_image_dir',
                    help='Relative path to the right camera image',
                    default='stereo_cameras/right')
  parser.add_option('--image_file_string', default='img_%04i.bmp',
                    help='Python format string to generate the image filename')
  parser.add_option('--annotations',
                    help='Relative path to the annotations file',
                    default='annotations.txt')
  parser.add_option('--output_dir',
                    help='Directory to output the results to')
  parser.add_option('--output_name', default=None,
                    help='Unique name for the outputs. Defaults to the estimator class')
  parser.add_option('--debug', action='store_true', default=False,
                    help='Run in debugging mode')

  # Sampling parameters
  parser.add_option('--min_region_width', type='int', default=64,
                    help='Width of the minimum region to evaluation in pixels')
  parser.add_option('--min_region_height', type='int', default=128,
                    help='Height of the minimum region to evaluation in pixels')
  parser.add_option('--win_stride', type='int', default=8,
                    help='When sampling, the stride in pixels for identifying regions')
  parser.add_option('--scale_stride', type='float', default=1.10,
                    help='When sampling, the scaling factor between levels')
  parser.add_option('--frame_subset_rate', type='int', default=1,
                    help='In order to run quicker, you can only run the detector on a subset of the frames. This specifies how often to run the detector. So, for example if it is 5, it will run the detector once every 5 frames.')

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
  parser.add_option('--laplacian_size', type='int',
                    help='Size of the laplacian aperture in pixels.',
                    default=3)
  parser.add_option('--hist_dist_type',
                    help='Type of distance metric to use to compare histograms',
                    default='chisq')
  parser.add_option('--estimator_scales',
                    help='Comma separated float specifying a set of scales to use for the estimator',
                    default='1.0')
  parser.add_option('--cascade_model_file',
                    help="File to load into the cascade detector. Works for integral cascade too.",
                    default="")
  parser.add_option('--img_scaling', type='float', default=0.5,
                    help="How much to scale the image if using ScaledDetectorWrapper")
  parser.add_option('--vu_thresh', type='float', default=None,
                    help='Threshold to pass the visual utility filter. If none, then only the maximum value is passed.')

  # For the high level hog detector
  parser.add_option('--hog_model_file',
                    help="File to load into the hog detector",
                    default="")
  parser.add_option('--hog_do_people', action='store_true',
                    dest='hog_do_people', default=True)
  parser.add_option('--no_hog_do_people', action='store_false',
                    dest='hog_do_people')
  parser.add_option('--hog_do_cache', action='store_true',
                    dest='hog_do_cache', default=False)

  (options, args) = parser.parse_args()

  rospy.init_node('RunHimaCascade', anonymous=True)

  inputDirs = HimaDataLoader.ParseInputDirs(options.input_dirs)

  # Run each of the directories
  for inputDir in inputDirs:
    if options.debug:
      ProcessOneDirectory(inputDir, options)
    else:
      p = Process(target=ProcessOneDirectory,
                  args=(inputDir, options))
      p.start()
      p.join()
