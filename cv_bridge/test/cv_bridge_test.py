#!/usr/bin/python
import roslib; roslib.load_manifest('cv_bridge')
import rospy
import unittest
from cv_bridge import cv_bridge
import numpy as np
import struct

class MatNDTest(unittest.TestCase):
  def setUp(self):
    self.mat = np.array(range(24), np.uint8)
    self.mat = np.reshape(self.mat, (2,3,4))

    self.assertEqual(self.mat[0][1][2], 6)
    self.assertEqual(self.mat[1][0][1], 13)
    self.assertEqual(self.mat[1][2][0], 20)

    self.bridge = cv_bridge.NumpyBridge()
    

  def test_RoundTripTestCopyNoConversion(self):
    msg = self.bridge.numpy_to_matmsg(self.mat)

    copyMat = self.bridge.matmsg_to_numpy(msg)

    self.assertEqual(self.mat.shape, copyMat.shape)

    self.assertTrue((self.mat == copyMat).all())

    self.assertEqual(copyMat[0][1][2], 6)
    self.assertEqual(copyMat[1][0][1], 13)
    self.assertEqual(copyMat[1][2][0], 20)

  def test_RoundTripTest32BitInt(self):
    startMat = np.array(self.mat, np.int32)
    msg = self.bridge.numpy_to_matmsg(startMat)
    copyMat = self.bridge.matmsg_to_numpy(msg)

    self.assertEqual(startMat.shape, copyMat.shape)
    self.assertEqual(copyMat.dtype, np.int32)
    self.assertTrue((startMat == copyMat).all())

  def test_RoundTripTestWithConversion(self):
    msg = self.bridge.numpy_to_matmsg(self.mat)

    copyMat = self.bridge.matmsg_to_numpy(msg, "32S")

    self.assertEqual(copyMat.dtype, np.int32)
    self.assertEqual(self.mat.dtype, np.uint8)

    self.assertEqual(self.mat.shape, copyMat.shape)

    self.assertTrue((self.mat == copyMat).all())

  def test_CheckMessageFormat(self):
    msg = self.bridge.numpy_to_matmsg(self.mat)

    self.assertEqual(msg.sizes, (2,3,4))
    self.assertEqual(msg.encoding, '8U')
    self.assertFalse(msg.is_bigendian)
    self.assertEqual(len(msg.data), 24)
    for i in range(24):
      self.assertEqual(msg.data[i], struct.pack('B', i))

if __name__ == '__main__':
  unittest.main()
