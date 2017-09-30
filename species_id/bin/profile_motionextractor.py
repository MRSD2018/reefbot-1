#!/usr/bin/python

import cv
import numpy as np
import cProfile
from optparse import OptionParser
from MotionExtractor import MotionExtractor
from EstimateAffineTransformFlow import AffineTransformEstimator
from EvolutionModel import GaussianEvolutionModel

def run_extractor(maxTime):  
  # Set up the objects that handle the parameters
  evolutionModel = GaussianEvolutionModel(10.0,
                                          10.0,
                                          0.1)
  transformEstimator = AffineTransformEstimator(100,
                                                1e-7,
                                                2.0,
                                                2)
  motionExtractor = MotionExtractor(transformEstimator,
                                    evolutionModel,
                                    0.6,
                                    12.0,
                                    2.0,
                                    0.5,
                                    200)

  # Now open the video for input
  #inStream = cv.CaptureFromFile('/data/mdesnoye/fish/media_videos/12-37-08_5.avi')
  inStream = cv.CaptureFromFile('/home/mdesnoyer/data/fish/tank_videos/12-37-08_5.avi')
  fps = cv.GetCaptureProperty(inStream, cv.CV_CAP_PROP_FPS);


  curTime = 0;
  cvFrame = cv.QueryFrame(inStream)
  while (cvFrame is not None):
    
    frame = np.asarray(cv.GetMat(cvFrame)).astype(np.float64) / 255.0
    motionExtractor.AddImage(frame, curTime)

    if (motionExtractor.RetreiveObjects() is not None and
        (curTime - motionExtractor.time() < 5.0/fps)):
      # Create a new frame showing the objects in red
      outFrame = frame
      outFrame[:,:,2] = outFrame[:,:,2]*0.6 + 0.4 * \
                        motionExtractor.RetreiveObjects().ToBinaryImageMask().astype(np.float64)

      outFrame *= 255.0

    cvFrame = cv.QueryFrame(inStream)
    curTime += 1.0/fps
    print curTime

    if curTime > maxTime:
      return

if __name__ == '__main__':
  cProfile.run('run_extractor(0.09)', 'motion_extractor.prof')
  
