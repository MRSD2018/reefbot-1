#!/usr/bin/python
import unittest
from EstimateAffineTransformFlow import *
import numpy as np
import math
from pylab import *

class TestSyntheticTransforms(unittest.TestCase):

  def setUp(self):
    self.image = np.zeros((20,30), np.float64)

    for i in range(3,14):
      self.image[i, (i+10)/2] = i / 20.

    for i in range(9,18):
      self.image[6, i] = 0.6

  def CreateAffineTransform(self, angle, xTranslation, yTranslation):
    return np.array(((math.cos(angle*math.pi/180.),
                      math.sin(angle*math.pi/180),
                      xTranslation),
                     (-math.sin(angle*math.pi/180.),
                      math.cos(angle*math.pi/180),
                      yTranslation),
                     (0,0,1)), np.float64)

  def assertMatrixAlmostEqual(self, a, b, places=7, msg=None):
    self.assertEqual(a.shape, b.shape)

    for i in range(a.shape[0]):
      for j in range(b.shape[1]):
        self.assertAlmostEqual(a[i,j], b[i,j], places=places, msg=msg)

  def test_SmallRotation(self):
    desiredTransform = self.CreateAffineTransform(1.0, 0, 0)

    warpedImage = AffineWarp(self.image, desiredTransform)[0]
    warpedImage[np.isnan(warpedImage)] = 0.0

    foundTransform = EstimateAffineTransformFlow(self.image, warpedImage,
                                                 sigma=0.2)

    self.assertMatrixAlmostEqual(desiredTransform, foundTransform, places=4)

  def test_LargerRotation(self):
    desiredTransform = self.CreateAffineTransform(5.0, 0, 0)

    warpedImage = AffineWarp(self.image, desiredTransform)[0]
    warpedImage[np.isnan(warpedImage)] = 0.0

    foundTransform = EstimateAffineTransformFlow(self.image, warpedImage,
                                                 sigma=0.2)

    self.assertMatrixAlmostEqual(desiredTransform, foundTransform, places=4)

  def test_SmallTranslation(self):
    desiredTransform = self.CreateAffineTransform(0, 2, -2)

    warpedImage = AffineWarp(self.image, desiredTransform)[0]
    warpedImage[np.isnan(warpedImage)] = 0.0

    foundTransform = EstimateAffineTransformFlow(self.image, warpedImage,
                                                 sigma=0.2)

    self.assertMatrixAlmostEqual(desiredTransform, foundTransform, places=4)

  def test_IdentityTransform(self):
    desiredTransform = self.CreateAffineTransform(0.0, 0, 0)

    warpedImage = AffineWarp(self.image, desiredTransform)[0]
    warpedImage[np.isnan(warpedImage)] = 0.0

    foundTransform = EstimateAffineTransformFlow(self.image, warpedImage)

    self.assertMatrixAlmostEqual(desiredTransform, foundTransform)

  def test_NoTransform(self):
    
    foundTransform = EstimateAffineTransformFlow(self.image, self.image)

    self.assertMatrixAlmostEqual(np.eye(3), foundTransform)
    
  
if __name__ == '__main__':
  unittest.main()
