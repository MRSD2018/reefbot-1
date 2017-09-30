#! /usr/bin/python
'''Runs an experiment that identifies people in a set of HIMA streams.

This script will output a series of bag files. Each bag file will have the following topics:

/parameters - List of Parameter messages one for each parameter of the run
/results - Sequence of FrameResult messages which specify the ground truth and the found locations of the people
'''
usage='RunHimaExperiment.py [options]'

import roslib; roslib.load_manifest('hima_experiment')
import rospy
import rosbag
from optparse import OptionParser
import os
import os.path
from std_msgs.msg import String
from std_msgs.msg import Duration
from sensor_msgs.msg import RegionOfInterest
from cascade_parts_detector.msg import DetectionArray
from cascade_parts_detector.srv import DetectObject
from hima_experiment.msg import FrameResult
from hima_experiment.msg import Parameter
from visual_utility.srv import FilterImage
from ParameterSetter import ParameterSetter
import cv
import cv_bridge
import subprocess
import copy
import random
import string
import threading
import collections
import HimaDataLoader

def GetRandomRunName():
  retval = ''.join(random.choice(string.letters) for i in xrange(10))
  rospy.loginfo('Name of this run is ' + retval)
  return retval

def StartPersonDetector(outputDir, namespace, modelFile, personCascade,
                        personThresh, parameterBag, bagTime):
  rospy.loginfo('Running the person detector')
  
  nodeName = '%s_person_detector' % namespace
  serviceName = '%s/detect_object_service' % nodeName
  parameters = ParameterSetter(nodeName,
                               [('model_file', modelFile),
                                ('thresh', personThresh),
                                ('do_cascade', personCascade)],
                               parameterBag, 'parameters', bagTime)

  proc = subprocess.Popen(['rosrun', 'cascade_parts_detector',
                           'CascadePartsDetectorNode',
                           '__name:=' + nodeName,
                           '__log:=%s/%s_log.out' % (outputDir, nodeName),
                           'detect_object_service:=%s' % serviceName,
                           'detect_object:=%s/detect_object' % nodeName,
                           'object_detected:=%s/object_detected' % nodeName,
                           'processing_time:=%s/processing_time' % nodeName])

  rospy.loginfo('Waiting for the person detector to come up')
  rospy.wait_for_service(serviceName,
                         timeout=6000)
  rospy.loginfo('Person detector is running')
  serviceProxy = rospy.ServiceProxy(serviceName, DetectObject)
  
  return (proc, parameters, serviceProxy)

def StartVisualUtilityFilter(outputDir, namespace, transformEstimator,
                             affineMaxIterations, minAffinePrecision,
                             affineScalingFactor, vuEstimator,
                             paretoThresh, distDecay, vuMosaic,
                             morphCloseSize, gaussSigma,
                             frameEstimator, frameExpansion,
                             framesize, minFrameArea, minEntropy,
                             parameterBag, bagTime):
  rospy.loginfo('Running the visual utility filter')
  
  nodeName = '%s_vufilter' % namespace
  serviceName = '%s/filter_image_service' % nodeName
  parameters = ParameterSetter(nodeName,
                               [('affine_max_iterations', affineMaxIterations),
                                ('min_affine_precision', minAffinePrecision),
                                ('affine_scaling_factor', affineScalingFactor),
                                ('vu_estimator', vuEstimator),
                                ('pareto_thresh', paretoThresh),
                                ('dist_decay', distDecay),
                                ('frame_estimator', frameEstimator),
                                ('frame_expansion', frameExpansion),
                                ('morph_close_size', morphCloseSize),
                                ('gauss_sigma', gaussSigma),
                                ('xframesize', framesize[0]),
                                ('yframesize', framesize[1]),
                                ('min_frame_area', minFrameArea),
                                ('min_entropy', minEntropy)],
                               parameterBag, 'parameters', bagTime)

  proc = subprocess.Popen(['rosrun', 'visual_utility',
                           'VisualUtilityFilterNode',
                           '__name:=' + nodeName,
                           '__log:=%s/%s_log.out' % (outputDir, nodeName),
                           'image:=%s/left_image' % nodeName,
                           'filtered/image:=%s/filtered/image' % nodeName,
                           'object_detected:=%s/object_detected' % nodeName,
                           'processing_time:=%s/processing_time' % nodeName,
                           'filter_image_service:=%s' % serviceName])

  rospy.loginfo('Waiting for the visual utility filter to startup')
  rospy.wait_for_service(serviceName,
                         timeout=6000)
  rospy.loginfo('Visual utility filter is running')
  serviceProxy = rospy.ServiceProxy(serviceName, FilterImage)
  
  return (proc, parameters, serviceProxy)

def AdjustDetectorResponse(detectorResponse, cameraInfo, fracResample):
  '''Changes the detectorResponse to be in the large image coordinates.'''
  retVal = copy.deepcopy(detectorResponse)

  for i in range(len(detectorResponse.detections)):
    if cameraInfo is not None:
      retVal.detections[i].x_offset += cameraInfo.roi.x_offset
      retVal.detections[i].y_offset += cameraInfo.roi.y_offset

    retVal.detections[i].height /= fracResample
    retVal.detections[i].width /= fracResample
    retVal.detections[i].x_offset /= fracResample
    retVal.detections[i].y_offset /= fracResample

  return retVal

def AdjustCameraInfo(cameraInfo, fracResample):
  if cameraInfo is None:
    return None

  retVal = copy.deepcopy(cameraInfo)
  retVal.x_offset /= fracResample
  retVal.y_offset /= fracResample
  retVal.height /= fracResample
  retVal.width /= fracResample
  return retVal

def WriteProcessingTimeToBag(msg, bag, bagLock, topic, time):
  bagLock.acquire()
  bag.write(topic, msg, time)
  bagLock.release()
  

def RunOneExperiment(frames, outputDir, resultFile, modelFile,
                     personCascade, personThresh, frameSubsetRate=1,
                     doVuFilter=False,
                     transformEstimator=None, affineMaxIterations=None,
                     minAffinePrecision=None, affineScalingFactor=None,
                     vuEstimator=None, paretoThresh=None, distDecay=None,
                     vuMosaic=None, morphCloseSize=0, gaussSigma=0.0,
                     frameEstimator=None,
                     frameExpansion=1.0, fracFramesize=1.0, minFrameArea=None,
                     minEntropy=None, fracResample=1.0):  
  runName = GetRandomRunName()

  cvBridge = cv_bridge.CvBridge()

  # Calculate the largest framesize
  exampleImage = cv.LoadImage(frames[0].imageFile)
  framesize = (exampleImage.width*fracFramesize,
               exampleImage.height*fracFramesize)

  startTime = frames[0].timestamp
  timeDiff = rospy.Time.now() - rospy.Time.from_sec(startTime)

  # Open up the bag which we will output stuff to
  outBag = rosbag.Bag(os.path.join(outputDir, resultFile), 'w')
  bagLock = threading.Lock()
  
  (personProc, personParams, detectorService) = StartPersonDetector(
    outputDir,
    runName,
    modelFile,
    personCascade,
    personThresh,
    outBag,
    rospy.Time.from_sec(startTime))
  try:
    if doVuFilter:
      (vuProc, vuParams, filterService) = StartVisualUtilityFilter(
        outputDir,
        runName,
        transformEstimator,
        affineMaxIterations,
        minAffinePrecision,
        affineScalingFactor,
        vuEstimator,
        paretoThresh,
        distDecay,
        vuMosaic,
        morphCloseSize,
        gaussSigma,
        frameEstimator,
        frameExpansion,
        framesize,
        minFrameArea,
        minEntropy,
        outBag,
        personParams.lastParameterTime)
    else:
      vuProc = None
      vuParams = None
      filterService = None

    try:
      # Register listeners that will record the processing time for each step
      procVuSub = rospy.Subscriber(
        '%s_vufilter/processing_time' % runName,
        Duration,
        lambda x: WriteProcessingTimeToBag(x, outBag, bagLock,
                                           'vufilter/processing_time',
                                           rospy.Time.now() - timeDiff))
      procDetectorSub = rospy.Subscriber(
        '%s_person_detector/processing_time' % runName,
        Duration,
        lambda x: WriteProcessingTimeToBag(x, outBag, bagLock,
                                           'person_detector/processing_time',
                                           rospy.Time.now() - timeDiff))
      
      try:
        curSeq = 0
        for frame in frames:
          cameraInfos = [None]
          if not os.path.exists(frame.imageFile):
            rospy.logerr('%s does not exists. Skipping' % frame.imageFile)
            continue
          cvImage = cv.LoadImageM(frame.imageFile)
          resizedImage = cv.CreateMat(int(cvImage.rows * fracResample),
                                      int(cvImage.cols * fracResample),
                                      cvImage.type)
          cv.Resize(cvImage, resizedImage)
          image = cvBridge.cv_to_imgmsg(resizedImage,
                                        'bgr8')
          image.header.stamp = rospy.Time.from_sec(frame.timestamp)
          image.header.seq = curSeq
          curSeq = curSeq + 1
          images = [image]

          # Run the visual utility filter
          if filterService is not None:
            filterResponse = filterService(image)

            images = filterResponse.images
            cameraInfos = filterResponse.camera_infos

          # See if we should skip sending the frame to the detector
          if curSeq % frameSubsetRate <> 0:
            continue

          # Initialize the result message
          frameResult = FrameResult()
          frameResult.header.stamp = rospy.Time.from_sec(frame.timestamp)
          frameResult.image = frame.imageFile
          for person in frame.peopleLocations.values():
              frameResult.groundTruth.append(person.BuildMsg())

          # Now find the people in each of the regions of interest
          for i in range(len(images)):
            image = images[i]
            cameraInfo = cameraInfos[i]
            
            # Now find the people for this region
            detectorResponse = detectorService(image)

            # Add to the detector response
            frameResult.detectorResponse.detections.extend(
              AdjustDetectorResponse(
                detectorResponse.detections, cameraInfo,
                fracResample).detections)
            frameResult.detectorResponse.scores.extend(
              detectorResponse.detections.scores)

            if cameraInfo is not None:
              frameResult.roi.append(AdjustCameraInfo(cameraInfo.roi,
                                                      fracResample))

          bagLock.acquire()
          outBag.write('results', frameResult, frameResult.header.stamp)
          bagLock.release()

      finally:
        procVuSub.unregister()
        procDetectorSub.unregister()        

    finally:
      if vuParams is not None: vuParams.RemoveParameters()
      if filterService is not None: filterService.close()
      if vuProc is not None: vuProc.send_signal(15)

  finally:
    personParams.RemoveParameters()
    detectorService.close()
    personProc.send_signal(15)
    outBag.close()

def RunControl(frames, outputDir, resultFile, modelFile,
               personCascade, personThresh, frameSubsetRate=1):
  RunOneExperiment(frames, outputDir, resultFile, modelFile,
                   personCascade, personThresh,
                   frameSubsetRate=frameSubsetRate, doVuFilter=False)

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
  parser.add_option('--do_lowres_experiment', action='store_true',
                    default=False,
                    help='Should we run the set of experiments where the frame is a lower resolution?')
  parser.add_option('--skip_vuexperiment', action='store_false',
                    default=True, dest='do_vuexperiment',
                    help='Should we skip the experiment with visual utility?')
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
  parser.add_option('--pareto_thresh', type='float',
                    help='Fraction of pixels to keep assuming pareto distribution of moving pixels',
                    default=0.03)
  parser.add_option('--dist_decay', type='float',
                    help='Sigmoid decay around the threshold',
                    default=2.0)

  # For the visual utility mosaic
  parser.add_option('--vu_mosaic',
                    help='String specifying the type of visual utility mosaic to use',
                    default='NULLVUMosaic')
  parser.add_option('--morph_close_size', type='int',
                    help='Size of the element to use when smoothing using a morphological close operation. ',
                    default=0)
  parser.add_option('--gauss_sigma', type='float',
                    help='How much gaussian blur to do on the frame when adding to the visual utility mosaic',
                    default=0.0)

  # For the frame estimator
  parser.add_option('--frame_estimator',
                    help='String specifying the frame estimator to us',
                    default='MaxPointConstantFramesize')
  parser.add_option('--frame_expansion',
                    help='Python code specifying the list frame magnifications to try',
                    default='[1.0]')
  parser.add_option('--frac_framesize',
                    help='When constant, python code specifying the fraction of the frame to send to the parts detector',
                    default='[1.0]')
  parser.add_option('--min_frame_area',
                    help='Minimum area for the frame to find',
                    default=200)
  parser.add_option('--min_entropy', default='[0.2]',
                    help='For a HighRelativeEntropy frame estimator, the entropy threshold to accept regions')

  
  # For the person detector
  parser.add_option('--model_file',
                    help='File specifying the model needed by the person detector',
                    default=None)
  parser.add_option('--person_cascade', default=False, action='store_true',
                    help="Use a cascade for the person detector?")
  parser.add_option('--person_thresh', default=0.0, type='float',
                    help='Threshold for the person detector')
  

  (options, args) = parser.parse_args()

  rospy.init_node('HimaExperiment', anonymous=True)

  minEntropies = eval(options.min_entropy)
  if not isinstance(minEntropies, collections.Iterable):
    minEntropies = [minEntropies]

  inputDirs = HimaDataLoader.ParseInputDirs(options.input_dirs)

  # Run each of the experiments
  for inputDir in inputDirs:
    dataset = os.path.basename(os.path.normpath(inputDir))

    frames = HimaDataLoader.LoadFrames(
      inputDir, options.left_image_dir,
      options.right_image_dir,
      options.annotations,
      imgFileString = options.image_file_string)
    
    for fracFramesize in eval(options.frac_framesize):
      for frameExpansion in eval(options.frame_expansion):
        for minEntropy in minEntropies:
          if options.do_vuexperiment:
            RunOneExperiment(frames,
                             options.output_dir,
                             ('results_%s_frame%s_expand%s_entropy%s.bag' %
                              (dataset, fracFramesize, frameExpansion,
                               minEntropy)),
                             options.model_file,
                             options.person_cascade,
                             options.person_thresh,
                             options.frame_subset_rate,
                             True, # doVuFilter
                             options.transform_estimator,
                             options.affine_max_iterations,
                             options.min_affine_precision,
                             options.affine_scaling_factor,
                             options.vu_estimator,
                             options.pareto_thresh,
                             options.dist_decay,
                             options.vu_mosaic,
                             options.morph_close_size,
                             options.gauss_sigma,
                             options.frame_estimator,
                             frameExpansion,
                             fracFramesize,
                             options.min_frame_area,
                             minEntropy
                           )
      if options.do_lowres_experiment:
        RunOneExperiment(frames,
                         options.output_dir,
                         ('results_%s_resize%s.bag' %
                          (dataset, fracFramesize)),
                         options.model_file,
                         options.person_cascade,
                         options.person_thresh,
                         frameSubsetRate=options.frame_subset_rate,
                         fracResample=fracFramesize
                         )

    RunControl(frames,
               options.output_dir,
               ('results_%s_control.bag' % (dataset)),
               options.model_file,
               options.person_cascade,
               options.person_thresh,
               frameSubsetRate=options.frame_subset_rate
               )

  
