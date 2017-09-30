import numpy as np
import scipy as sp
import scipy.ndimage as ndimage

class EvolutionModel:
  '''An abstract evolution model for probability distributions.'''
  def __init__(self):
    pass

  def GenerateEstimate(self, transformEstimator, transform, time, shape):
    '''Generate the current estimate of the probability distrubtion.

    Inputs:
    transformEstimator - object used to do transforms on the image
    transform - The transform from the last time AddDistribution was called to now
    time - The time now
    shape - The desired shape of the distribution model

    Outputs:
    A numpy image of the current estimated probability distribution
    '''
    raise NotImplementedException()

  def AddDistribution(self, distribution, time):
    '''Add a measured probability distrubtion to the model/

    Inputs:
    distribution - numpy image of the probability distribution
    time - the time the distribution is valid in seconds
    '''
    raise NotImplementedException()

class GaussianEvolutionModel(EvolutionModel):
  '''An evolution model that assumes gaussian dispersion over time.

  The resulting model is:
  p = (1-alpha)*conv(lastDistribution, gauss(xsigma*t, ysigma*t)) +
  alpha * 1/numPixels

  '''
  def __init__(self, xsigma, ysigma, alpha):
    self.xsigma = xsigma
    self.ysigma = ysigma
    self.alpha = alpha
    self.time = None
    self.lastDistribution = None

  
  def GenerateEstimate(self, transformEstimator, transform, time):
    if self.lastDistribution is None:
      return 0.5

    dt = time - self.time
    estimate = ndimage.filters.gaussian_filter(
      self.lastDistribution,
      (self.ysigma*dt, self.xsigma*dt),
      order=0,
      mode='constant')

    warpedEstimate = transformEstimator.ApplyTransform(estimate, transform)

    return self.alpha + (1-self.alpha)*warpedEstimate

  def AddDistribution(self, distribution, time):
    self.lastDistribution = distribution
    self.time = time
    
                                               
