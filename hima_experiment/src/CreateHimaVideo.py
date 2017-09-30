#!/usr/bin/python
'''Creates a video where each frame is annotated with the subframe, the true person and the detected person.

True person is red
Detected person is green
Frame is yellow
'''
usage='CreateHimaVideo.py'

import roslib; roslib.load_manifest('hima_experiment')
import rospy
from pylab import *
import numpy as np
from optparse import OptionParser
import math
import re
import os.path
import cv
import rosbag
import subprocess
from DetectorMetrics import *
from ParameterSetter import ParameterSetter
from visual_utility.srv import FilterImage
import cv_bridge

def PointInRoi(roi, x, y):
  if roi.height == 0 or roi.width == 0:
    return True
  return (roi.x_offset <= x and roi.y_offset <= y and
          x <= roi.x_offset + roi.width and
          y <= roi.y_offset + roi.height)

def FixOffsets(bbox):
  # On some of the datasets, the bounding box can have negative
  # offsets which get encoded as 2^32-val. So fix that.

  retval = bbox
  if retval.x_offset > (1<<31):
    retval.width += retval.x_offset - (1<<32)
    retval.x_offset = 0
  if retval.y_offset > (1<<31):
    retval.height += retval.y_offset - (1<<32)
    retval.y_offset = 0

  return retval

def NormalizeParamName(paramName):
  '''Converts parameter name from camelcase to underscores.'''
  s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', paramName)
  return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

def ToNumIfPossible(v):
  '''Converts a value to a number if it is possible.'''
  try:
    return float(v)
  except ValueError: pass

  return v

def RunVisualUtilityFilter(bag, nodeName):

  # First go through the bag and get the parameters
  paramList = [(NormalizeParamName(msg.name.data),
                ToNumIfPossible(msg.value.data))
               for topic, msg, t
               in bag.read_messages(topics=['parameters'])]
  paramList.append(('visual_debugging', True))
  parameters = ParameterSetter(nodeName, paramList)

  # Start the visual utility filter
  serviceName = '%s/filter_image_service' % nodeName

  rospy.loginfo('Running the visual utility filter')
  proc = subprocess.Popen(['rosrun', 'visual_utility',
                           'VisualUtilityFilterNode',
                           '__name:=' + nodeName,
                           'image:=%s/left_image' % nodeName,
                           'filtered/image:=%s/filtered/image' % nodeName,
                           'object_detected:=%s/object_detected' % nodeName,
                           'processing_time:=%s/processing_time' % nodeName,
                           'debug/vuestimate:=%s/debug/vuestimate' % nodeName,
                           'debug/mosaic:=%s/debug/mosaic' % nodeName,
                           'debug/frameImage:=%s/debug/frameImage' % nodeName,
                           'filter_image_service:=%s' % serviceName])

  rospy.loginfo('Waiting for the visual utility filter to startup')
  rospy.wait_for_service(serviceName,
                         timeout=6000)
  rospy.loginfo('Visual utility filter is running')
  serviceProxy = rospy.ServiceProxy(serviceName, FilterImage)

  return (proc, parameters, serviceProxy)

def DrawRectangles(image, msg):

  # Draw rectangles for the ground truth
  for personLocation in msg.groundTruth:
    if personLocation.annotationType < 0:
      continue

    color = cv.RGB(0,0,255)
    # See if we detected this person and if so, set the color to
    # blue. Otherwise the color is red
    if not personLocation.targetId in foundTargets:
      foundCurTarget = False
      for detection in msg.detectorResponse.detections:
        if detectorMetric.BoxesMatch(personLocation.bbox, detection):
          foundCurTarget = True
          break
      if foundCurTarget:
        foundTargets = foundTargets | set([personLocation.targetId])
      else:
        color = cv.RGB(255,0,0)
          
      
      bbox = FixOffsets(personLocation.bbox)      
      
    cv.Rectangle(image,
                 (bbox.x_offset, bbox.y_offset),
                 (bbox.x_offset + bbox.width,
                  bbox.y_offset + bbox.height),
                 color,
                 2) # thickness

  # Draw rectangles for the people found
  for detection in msg.detectorResponse.detections:
    bbox = detection
    cv.Rectangle(image,
                 (bbox.x_offset, bbox.y_offset),
                 (bbox.x_offset + bbox.width,
                  bbox.y_offset + bbox.height),
                 cv.RGB(0, 255, 0),
                 2) # thickness

  # Draw the frame that was passed to the person detector
  for roi in msg.roi:
    if roi.height > 0 and roi.width > 0:
      cv.Rectangle(image,
                   (roi.x_offset, roi.y_offset),
                   (roi.x_offset + roi.width,
                    roi.y_offset + roi.height),
                   cv.RGB(255, 255, 0),
                   2) # thickness

  return image
  

if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('--input', '-i', dest='input',
                    help='Input bag to use',
                    default=None)
  parser.add_option('--output', '-o', dest='output',
                    help='Filename of the output video',
                    default=None)
  parser.add_option('--score_thresh', type='float', default=0,
                    help='Threshold for the detection score to include result')
  parser.add_option('--do_object_memory', action='store_true',
                    dest='do_object_memory', default=False,
                    help='Add an analysis that assumes objects stay where they were if they were not looked at')
  parser.add_option('--width_ratio', type='float', default=0.41,
                    help='Standard ratio of width to height for a person. Default value is from Dollar et. al. 2001')
  parser.add_option('--min_overlap', type='float',
                    help='Minimum fraction overlap for a target to be correct',
                    default=0.5)
  parser.add_option('--do_vu_debugging', action='store_true',
                    dest='do_vu_debugging', default=False,
                    help='If true, video becomes four frames with debugging info for looking at visual utility. This will be a lot slower!')


  (options, args) = parser.parse_args()

  detectorMetric = BoundingBoxOverlap(options.score_thresh,
                                      options.do_object_memory,
                                      options.width_ratio,
                                      options.min_overlap)

  # Open the input bag to read
  bag = rosbag.Bag(options.input)

  # Figure out the framesize
  exampleMsg = bag.read_messages(topics=['results']).next()[1]
  exampleImage = cv.LoadImage(exampleMsg.image)
  frameSize = (exampleImage.width, exampleImage.height)

  # Open the video for output
  videoOutput = cv.CreateVideoWriter(options.output,
                                     cv.CV_FOURCC('M', 'P', 'E', 'G'),
                                     30.0, # fps needs to be 30 for MPEG
                                     frameSize,
                                     1) # is_color

  vuProc = None
  vuParams = None
  vuService = None
  if options.do_vu_debugging:
    vuProc, vuParams, vuService = RunVisualUtilityFilter(bag, 'vufilter')

  cvBridge = cv_bridge.CvBridge()

  try:
    # Now go through the frames in the bag and make the video
    nFrames = 0
    lastMsg = None
    foundTargets = set()
    for topic, msg, t in bag.read_messages(topics=['results']):
      if topic <> 'results':
        continue
    
      # Strip out those detections less than the score threshold
      validDetections = []                                       
      for i in range(len(msg.detectorResponse.scores)):
        if msg.detectorResponse.scores[i] > options.score_thresh:
          validDetections.append(i)
      msg.detectorResponse.detections = [
        msg.detectorResponse.detections[x] for x in validDetections]
    
      # Add in detections from the last frame in regions we didn't run the
      # detector on this time.
      if options.do_object_memory:
        if lastMsg is not None:
          for detection in lastMsg.detectorResponse.detections:
            inRoi = False
            for roi in msg.roi:
              if PointInRoi(roi,
                            detection.x_offset +
                            detection.width / 2,
                            detection.y_offset +
                            detection.height / 2):
                inRoi = True
                break
            if not inRoi:
              msg.detectorResponse.detections.append(detection)
        
        lastMsg = msg
    
      image = cv.LoadImage(msg.image)

      # If we're debugging the visual utility, get the images from
      # along the way
      vuImage = None
      mosaicImage = None
      frameImage = None
      vuResponse = None
      if vuService is not None:
        imgMsg = cvBridge.cv_to_imgmsg(image, 'bgr8')
        vuResponse = vuService(imgMsg)
        if vuResponse.debug_vu_estimate.height > 0:
          vuImage = cvBridge.imgmsg_to_cv(vuResponse.debug_vu_estimate, 'bgr8')
        if vuResponse.debug_mosaic.height > 0:
          mosaicImage = cvBridge.imgmsg_to_cv(vuResponse.debug_mosaic, 'bgr8')
        if vuResponse.debug_framing.height > 0:
          frameImage = cvBridge.imgmsg_to_cv(vuResponse.debug_framing, 'bgr8')
        

      image = DrawRectangles(image, msg)

      outputFrame = cv.CreateMat(frameSize[0], frameSize[1],
                                 cv.CV_8UC3)
      cv.SetZero(outputFrame)
      if options.do_vu_debugging:
        quarterFrame = [x/2 for x in frameSize]
        smallFrame = cv.CreateMat(quarterFrame[0], quarterFrame[1],
                                  cv.CV_8UC3)
        # The original frame
        cv.Resize(image, smallFrame)
        sub = cv.GetSubRect(outputFrame, (0, 0, quarterFrame[1],
                                          quarterFrame[0]))
        cv.Copy(smallFrame, sub)

        # The visual utility
        if vuImage is not None and cv.GetSize(vuImage)[0] > 0:
          cv.Resize(vuImage, smallFrame)
          sub = cv.GetSubRect(outputFrame, (quarterFrame[1]-1, 0,
                                            quarterFrame[1],
                                            quarterFrame[0]))
          cv.Copy(smallFrame, sub)

        # The mosaic image
        if mosaicImage is not None and cv.GetSize(mosaicImage)[0] > 0:
          cv.Resize(mosaicImage, smallFrame)
          sub = cv.GetSubRect(outputFrame, (0, quarterFrame[0]-1,
                                            quarterFrame[1],
                                            quarterFrame[0]))
          cv.Copy(smallFrame, sub)

        # The frame image
        if frameImage is not None and cv.GetSize(frameImage)[0] > 0:
          for cameraInfo in vuResponse.camera_infos:
            roi = cameraInfo.roi
            if roi.height > 0 and roi.width > 0:
              cv.Rectangle(frameImage,
                           (roi.x_offset, roi.y_offset),
                           (roi.x_offset + roi.width,
                            roi.y_offset + roi.height),
                           cv.RGB(255, 255, 0),
                           2) # thickness
          
          cv.Resize(frameImage, smallFrame)
          sub = cv.GetSubRect(outputFrame, (quarterFrame[1]-1,
                                            quarterFrame[0]-1,
                                            quarterFrame[1],
                                            quarterFrame[0]))
          cv.Copy(smallFrame, sub)
                                          
      else:
        outputFrame = image
      
      cv.WriteFrame(videoOutput, cv.GetImage(outputFrame))

      nFrames = nFrames + 1
      if nFrames % 50 == 0:
        rospy.loginfo('Processed %i frames' % nFrames)

  finally:
    if vuParams is not None: del vuParams
    if vuService is not None: vuService.close()
    if vuProc is not None: vuProc.send_signal(15)

  del videoOutput
  
                   

  
