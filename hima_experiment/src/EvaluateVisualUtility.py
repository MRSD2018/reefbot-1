#!/usr/bin/python
'''Evaluates the visual utility for a set of images in HIMA format

This script will output a series of bag files. Each bag file will have the following topics:

/parameters - List of Parameter messages one for each parameter of the run
/results - Sequence of VisualUtilityEstimation messages specifying the scores
'''
usage='EvaluateVisualUtility.py [options]'

import roslib; roslib.load_manifest('hima_experiment')
import rospy
import rosbag
import gc
from optparse import OptionParser
from ParameterSetter import ParameterSetter
from objdetect_msgs.srv import DetectObjectService
from sensor_msgs.msg import RegionOfInterest
from objdetect_msgs.msg import DetectObject
from hima_experiment.msg import VisualUtilityEstimation
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

def StartVisualUtilityEstimator(outputDir, namespace, outBag, bagTime, options):
  rospy.loginfo('Running the Visual Utility Estimator')

  nodeName = '%s_vu_estimator' % namespace
  serviceName = '%s/detect_object_service' % nodeName
  parameterList = [('affine_max_iterations', options.affine_max_iterations),
                   ('min_affine_precision', options.min_affine_precision),
                   ('affine_scaling_factor', options.affine_scaling_factor),
                   ('vu_estimator', options.vu_estimator),
                   ('vu_estimator_wrapper', options.vu_estimator_wrapper),
                   ('pareto_thresh', options.pareto_thresh),
                   ('dist_decay', options.dist_decay),
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
                           'vu_objdetect_service:=%s' % serviceName],
                          env=vuEnv)

  rospy.loginfo('Waiting for the visual utility estimator to startup')
  rospy.wait_for_service(serviceName,
                         timeout=6000)
  rospy.loginfo('Visual utility filter is running')
  serviceProxy = rospy.ServiceProxy(serviceName, DetectObjectService)
  
  return (proc, parameters, serviceProxy)
  

def GetRandomRunName():
  retval = ''.join(random.choice(string.letters) for i in xrange(10))
  rospy.loginfo('Name of this run is ' + retval)
  return retval

def RescaleRegionOfInterest(roi, scale):
  if scale is None or scale >= 1.0:
    return roi
  roi.x_offset /= scale
  roi.y_offset /= scale
  roi.height /= scale
  roi.width /= scale
  return roi

def CalculateVisualUtilityForOneDataset(frames, outputDir, resultFile,
                                        imageScale, options):
  runName = GetRandomRunName()
  cvBridge = cv_bridge.CvBridge()

  # Open up the bag which we will output stuff to
  outBag = rosbag.Bag(os.path.join(outputDir, resultFile), 'w', 'bz2')

  (vuProc, vuParams, vuService) = StartVisualUtilityEstimator(outputDir, runName, outBag,
                                                              rospy.Time.from_sec(frames[0].timestamp), options)
  regions = []
  imageShape = None
  try:
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
      if imageScale is not None:
        scaledImage = cv2.resize(cvImage, (cvImage.cols * imageScale,
                                           cvImage.rows * imageScale))
      else:
        scaledImage = cvImage
        
      image = cvBridge.cv_to_imgmsg(scaledImage,
                                    'bgr8')
      image.header.stamp = rospy.Time.from_sec(frame.timestamp)
      image.header.seq = curSeq

      if imageShape is None or imageShape <> scaledImage.shape:
        regions = DetectionUtils.GenerateSamplingRegions(scaledImage,
                                                         options.min_region_width,
                                                         options.min_region_height,
                                                         options.win_stride,
                                                         options.scale_stride)
        imageShape = scaledImage.shape

      # Calculate the visual utility for each region
      request = DetectObject()
      request.image = image
      request.regions = regions
      response = vuService(request)

      if (len(response.detections.detections) <> len(regions)):
        rospy.logerr('Did not receive scores for all the regions in image: %s' % frame.imageFile)

      # Build up the result message to store
      resultMsg = VisualUtilityEstimation()
      resultMsg.header.stamp = rospy.Time.from_sec(frame.timestamp)
      resultMsg.header.seq = curSeq - 1
      resultMsg.image = frame.imageFile
      resultMsg.processing_time = response.processing_time
      resultMsg.regions = [x.mask.roi for x in response.detections.detections]
      resultMsg.scores = [x.score for x in response.detections.detections]
      resultMsg.overlaps = [frame.GetMaxOverlap(
        RescaleRegionOfInterest(x, imageScale))
                            for x in resultMsg.regions]

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
  parser.add_option('--image_scales', default=None,
                    help='Python expression specifying the scales of the image to sample. For instance [1.0, 0.8] will calculate the visual utility for full sized images and images that are 80% size')

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
                    

  (options, args) = parser.parse_args()

  rospy.init_node('EvaluateVisualUtility', anonymous=True)

  inputDirs = HimaDataLoader.ParseInputDirs(options.input_dirs)

  outputName = options.output_name
  if outputName is None:
    outputName = options.vu_estimator

  # Run each of the directories
  for inputDir in inputDirs:
    dataset = os.path.basename(os.path.normpath(inputDir))

    frames = HimaDataLoader.LoadFrames(
      inputDir, options.left_image_dir,
      options.right_image_dir,
      options.annotations,
      imgFileString = options.image_file_string)

    if options.image_scales is None:
      scales = [None]
    else:
      scales = eval(options.image_scales)

    for scale in scales:
      if scale is None:
        outputFn = 'vu_%s_%s.bag' % (outputName, dataset)
      else:
        outputFn = 'vu_%s_%s_scale%g.bag' % (outputName, dataset, scale)
        
      CalculateVisualUtilityForOneDataset(frames,
                                          options.output_dir,
                                          outputFn,
                                          scale,
                                          options)
    del frames
    gc.collect()
