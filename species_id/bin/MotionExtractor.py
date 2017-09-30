'''Class that identifies objects that move significantly relative to
the background in am image sequence.

Author: Mark Desnoyer (markd@cmu.edu)
Date: June 2011
'''
import numpy as np
import color
import roslib; roslib.load_manifest('species_id')
import rospy
import EstimateAffineTransformFlow
import math
import Blob
from pylab import *

class MotionExtractor:
  COLOR_GRAY=0
  COLOR_RGB=1
  COLOR_LAB=2
  
  def __init__(self, transformEstimator, evolutionModel, alpha,
               distThreshold, distDecay, confidenceThreshold=0.5,
               minObjSize=200, colorSpace=COLOR_LAB):
    '''Constructor

    Inputs:
    
    transformEstimator - Object that can estimate the transform from one frame to another and apply the discovered transform to a new frame. See AffineTransformEstimator.
    evolutionModel - Object that converts the previous frame\'s probability distribution into a prior for the current frame.
    alpha - Mixing parameter for combining the prior estimate of the motion with the current measurement. Smaller biases for the prior
    distThreshold - Theshold for the distance in between two frames that signifies an object that is moving enough. Modeled as a sigmoid function
    distDecay - How quickly the threshold decays in the sigmoid function
    confidenceThreshold - Overall convidence threshold to identify an object
    minObjSize - Minimum size in pixels for an object to be considered
    colorSpace - The colorspace used for measuing distances
    '''
    self.transformEstimator = transformEstimator
    self.evolutionModel = evolutionModel
    self.alpha = alpha
    self.distThreshold = distThreshold
    self.distDecay = distDecay
    self.confidenceThreshold = confidenceThreshold
    self.minObjSize = minObjSize

    self.objects = None
    self.time_ = -1.0
    self.colorSpace=colorSpace

    self.lastBwImage = None
    self.lastColorImage = None
    

  def AddImage(self, image, time):
    '''Adds a new image to the motion model.

    Inputs:
    image - An image as a numpy nxmxc array. If it is a color image, assumes RGB format.
    time - Time that the image was taken in seconds since the epoch

    Outputs:
    None
    '''
    bwImage = None
    colorImage = None

    # Find the grescale portion of the image
    if image.shape[2] == 1:
      # Image is grayscale
      bwImage = image.copy()
    elif image.shape[2] == 3:
      # Image is RGB
      bwImage = 0.2989*image[:,:,0] + 0.5870*image[:,:,1] + \
                0.1140*image[:,:,2]
    else:
      rospy.logerr("Image must be RGB or greyscalie")
      return

    # Figure out the desired color space for the color image
    if self.colorSpace == self.COLOR_GRAY:
      colorImage = bwImage
    elif image.shape[2] <> 3:
      rospy.logerr("Image must be RGB")
      return
    elif self.colorSpace == self.COLOR_RGB:
      colorImage = image.copy()
    elif self.colorSpace == self.COLOR_LAB:
      colorImage = color.rgb2lab(image)

    try:
      if self.lastBwImage is not None and self.lastColorImage is not None:
        # Calculate the geometric transform to convert the last image to
        # the current one
        transform = self.transformEstimator.EstimateTransform(
          self.lastBwImage, bwImage)

        # Warp the older color image to the current image space
        warpedImage = self.transformEstimator.ApplyTransform(
          self.lastColorImage, transform)

        # Calculate the distance between the two frames registered together
        diff = colorImage - warpedImage
        dist = np.sqrt(np.sum(diff*diff, 2))

        # Now calculate the probability of there being an object given
        # the distance measured between the two frames. We do this
        # using a sigmoid function to create a smooth threshold
        pObjGivDist = 1 / (1 + np.exp(-(dist - self.distThreshold) /
                                      self.distDecay))

        # Next retreive the probability of an object at a given
        # location given the measurement at the last time step. We do
        # this using the evolutionModel
        pObjGivPrior = self.evolutionModel.GenerateEstimate(
          self.transformEstimator, transform, time)

        # Calculate the probability that an object is at a location by
        # combining linearily the two conditional probabilities
        pObj = self.alpha*pObjGivDist + (1-self.alpha)*pObjGivPrior

        # Record the probability estimate in the evolution model
        self.evolutionModel.AddDistribution(pObj, time)

        # Now threshold the probability and find the objects
        self.objects = Blob.BlobResult()
        self.objects.FindBlobs(pObj > self.confidenceThreshold,
                               minimumSize = math.sqrt(self.minObjSize))

        
    except EstimateAffineTransformFlow.DidNotConvergeExecption:
      pass

    finally:
      self.lastBwImage = bwImage
      self.lastColorImage = colorImage
      self.time_ = time
        

  def time(self):
    '''Returns the time that the last useful image was taken.'''
    return self.time_

  def RetreiveObjects(self):
    '''Retreives the known objects as a Blob.BlobResult object.'''
    return self.objects
  
