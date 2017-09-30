#!/usr/bin/python
usage='highlight_motion.py [options] <input movie> <output movie>'

import cv
import numpy as np
from optparse import OptionParser
from MotionExtractor import MotionExtractor
from EstimateAffineTransformFlow import AffineTransformEstimator
from EvolutionModel import GaussianEvolutionModel

if __name__ == '__main__':
  # All of the command line flags
  parser = OptionParser(usage=usage)

  parser.add_option('--gauss_xsigma', type="float",
                    help='Size of the gaussian in the x direction',
                    default=10.0)
  parser.add_option('--gauss_ysigma', type="float",
                    help='Size of the gaussian in the y direction',
                    default=10.0)
  parser.add_option('--gauss_alpha', type="float",
                    help='Mixing parameter for the gaussian vs. uniform evolution model',
                    default=0.1)
  parser.add_option('--affine_iter', type="int",
                    help='Maximum number of iterations for the affine discovery',
                    default=100)
  parser.add_option('--affine_precision', type="float",
                    help='Desired precision of the affine matrix',
                    default=1e-7)
  parser.add_option('--affine_scaling', type="float",
                    help='Factor to shrink the image when finding the affine warp',
                    default=4.0)
  parser.add_option('--affine_smoothing', type="int",
                    help='Factor to smooth the image when finding the affine warp',
                    default=2)
  parser.add_option('--motion_alpha', type="float",
                    help='Fraction of the probability of seeing the object now to use',
                    default=0.6)
  parser.add_option('--dist_threshold', type="float",
                    help='Threshold for the distance being identified as an object',
                    default=12.0)
  parser.add_option('--dist_decay', type="float",
                    help='How quickly the threshold decays in the sigmoid',
                    default=2.0)
  parser.add_option('--confidence_threshold', type='float',
                    help='Overall confidence threshold to identify an object',
                    default=0.5)
  parser.add_option('--min_obj_size', type='float',
                    help='Minimum size in pixels for an object to be considered',
                    default=200
                    )

  (options, args) = parser.parse_args()

  inputFile = args[0]
  outputFile = args[1]

  # Set up the objects that handle the parameters
  evolutionModel = GaussianEvolutionModel(options.gauss_xsigma,
                                          options.gauss_ysigma,
                                          options.gauss_alpha)
  transformEstimator = AffineTransformEstimator(options.affine_iter,
                                                options.affine_precision,
                                                options.affine_scaling,
                                                options.affine_smoothing)
  motionExtractor = MotionExtractor(transformEstimator,
                                    evolutionModel,
                                    options.motion_alpha,
                                    options.dist_threshold,
                                    options.dist_decay,
                                    options.confidence_threshold,
                                    options.min_obj_size)

  # Now open the video for input
  inStream = cv.CaptureFromFile(inputFile)

  fps = cv.GetCaptureProperty(inStream, cv.CV_CAP_PROP_FPS);
  
  # Next open the file for output
  outStream = cv.CreateVideoWriter(
    outputFile,
    cv.CV_FOURCC('P', 'I', 'M', '1'),
    fps,
    (cv.GetCaptureProperty(inStream, cv.CV_CAP_PROP_FRAME_WIDTH),
     cv.GetCaptureProperty(inStream, cv.CV_CAP_PROP_FRAME_HEIGHT)),
    1) # is_color


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
      cv.WriteFrame(outStream, cv.GetImage(cv.fromarray(frame.astype(np.uint8()))))

    cvFrame = cv.QueryFrame(inStream)
    curTime += 1.0/fps

    print 'Processed %f seconds' % curTime
