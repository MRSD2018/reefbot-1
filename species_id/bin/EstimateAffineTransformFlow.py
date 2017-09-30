# Estiamtes an affine transform between two images using an iterative
# optical flow process.
#
# Author: Mark Desnoyer (markd@cmu.edu)
# Date: June 2011
import numpy as np
import scipy as sp
import numpy.linalg as linalg
import scipy.ndimage as spimage
from pylab import *
from scipy.interpolate import RectBivariateSpline

class DidNotConvergeExecption(Exception): pass

class InvalidParameters(Exception): pass

class AffineTransformEstimator:
  def __init__(self, maxIterations=100, minPrecision=1e-9, imageScaling=1.0,
               sigma=2.0) :
    self.maxIterations = maxIterations
    self.minPrecision = minPrecision
    self.imageScaling = imageScaling
    self.sigma = sigma

  def EstimateTransform(self, image1, image2):
    '''Estimates the affine transform matrix from image1 to image2.'''
    return EstimateAffineTransformFlow(image1, image2,
                                       self.maxIterations,
                                       self.minPrecision,
                                       self.imageScaling,
                                       self.sigma)

  def ApplyTransform(self, image, matrix):
    '''Applies a transform returned by EstimateTransform to the image.'''
    return AffineWarp(image, matrix)[0]

def AffineWarp(image, warpMatrix, splineObj=None):
  '''Warps an image using a 3x3 affine warp matrix.


  Locations outside are NaN

  This is done more manually because the
  scipy.ndimage.affine_transform was not accurate enough.

  Inputs:
  image - A numpy array for the image
  warpMatrix - A 3x3 affine transform matrix. The last row should be 0 0 1
  splineObj - Optional object that stores the splines already calculated. If you are warping the same image multiple times, it will be faster to keep this object and pass it back

  Outputs:
  (warpedImage, interpFunc)
  '''
  if image.ndim == 2:
    (nRows, nCols) = image.shape
    nChans = 1
    image = image.reshape(nRows, nCols, nChans)
  else:
    (nRows, nCols, nChans) = image.shape

  # Make sure that it is an affine warp matrix
  assert((warpMatrix[2, :] == np.array((0, 0, 1))).all())

  inputSplineObj = splineObj
  
  xi, yi = np.meshgrid(np.arange(nCols), np.arange(nRows))
  xi = xi.ravel()
  yi = yi.ravel()
  outCoords = np.dot(linalg.inv(warpMatrix),
                     np.vstack((xi, yi, np.ones((nRows*nCols)))));

  # Do the warping one channel at a time
  warpedImage = np.ndarray(shape=image.shape, dtype=image.dtype)
  for i in range(nChans):
    if inputSplineObj is None:
      splineObj = RectBivariateSpline(np.arange(nCols), np.arange(nRows),
                                      image[:,:,i].T,
                                      kx=1, ky=1)

    warpedImage[:,:,i].flat = splineObj.ev(outCoords[0,:], outCoords[1,:])
  return (warpedImage.squeeze(), splineObj)

  

def imresize(a, scalingFactor, **kw):
  if abs(scalingFactor-1.0) < 1e-10:
    return a
  return spimage.affine_transform(a,
                                  [1./scalingFactor, 1./scalingFactor],
                                  output_shape=[round(a.shape[0] * scalingFactor),
                                                round(a.shape[1] * scalingFactor)],
                                  **kw)

def EstimateAffineTransformFlow(srcImage, destImage, maxIterations=100,
                                minPrecision=1e-9, imageScaling=1.0,
                                sigma=2.0):
  '''Estimate the 3x3 affine transform from srcImage to destImage using optical flow.

  srcImage - The first image
  destImage - The second image
  maxIterations - Maximum number of iterations to perform
  minPrecision - Stopping criteria for how stable the transform must be
  imageScaling - Factor to shrink the images to make computation faster
  sigma - Parameter for the gaussian smoothing of the image to make it more robust

  returns: A 3x3 affine transform matrix if one was found.

  throws: DidNotConvergeException - If the estimation process failed
  '''

  if srcImage.shape <> destImage.shape:
    raise InvalidParameters('srcImage and destImage are not the same size');

  if srcImage.ndim <> 2:
    raise InvalidParameters('Must be a greyscale image')

  # Convert to 64bit if necessary
  if srcImage.dtype <> np.float64:
    srcImage = srcImage.astype(np.float64)
  if destImage.dtype <> np.float64:
    destImage = destImage.astype(np.float64)

  # Scale the images to make computation faster
  im1 = imresize(srcImage, 1./imageScaling, order=1, prefilter=False);
  im2 = imresize(destImage, 1./imageScaling, order=1, prefilter=False);

  (nRows, nCols) = im1.shape

  # Smooth out the images
  im1 = spimage.gaussian_filter(im1, sigma, 0, mode='constant')
  im2 = spimage.gaussian_filter(im2, sigma, 0, mode='constant')

  # Calculate the x and y derivatives of the image
  Ix = spimage.filters.convolve1d(im2, (0.5, 0, -0.5), 1, mode='constant',
                                  origin=0)
  Ix = Ix.ravel()
  Iy = spimage.filters.convolve1d(im2, (0.5, 0, -0.5), 0, mode='constant',
                                  origin=0)
  Iy = Iy.ravel()

  # Get the coordinates
  x, y = np.meshgrid(range(nCols), range(nRows))
  x = x.ravel()
  y = y.ravel()

  # Build up the A matrix
  A = np.transpose(np.array((x*Ix,
                             y*Ix,
                             Ix,
                             x*Iy,
                             y*Iy,
                             Iy), np.float64))
  aInv = linalg.pinv(A)

  deltaP = np.array((minPrecision + 1))
  M = np.eye(3, 3, dtype=np.float64)
  iterations = 0;
  splineObj = None

  while iterations < maxIterations and np.abs(deltaP).max() > minPrecision:
    # Warp im1 using the current M matrix
    (warpedImage, splineObj) = AffineWarp(im1, linalg.inv(M),
                                          splineObj=splineObj)
    validIndex = np.flatnonzero(np.isfinite(warpedImage))

    It = (im2-warpedImage).ravel()[validIndex]

    # Calculate the next transform in the iteration
    deltaP = np.dot(aInv[:, validIndex], It)

    newM = np.array(((1.+deltaP[0], deltaP[1], deltaP[2]),
                     (deltaP[3], 1.+deltaP[4], deltaP[5]),
                     (0., 0., 1.)), dtype=np.float64)

    M = np.dot(newM,M)

    iterations += 1

  M = linalg.inv(M)

  # Now scalie the transformation matrix because we potentially shrunk
  # the images for the calculatation and we need to go back to the
  # correct coordinates
  M[0:2,2] *= imageScaling

  if iterations >= maxIterations:
    raise DidNotConvergeExecption()

  return M
  
  
  
