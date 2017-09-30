#!/usr/bin/python

from HimaDataLoader import *
import unittest

class PersonLocationTest(unittest.TestCase):
  def setUp(self):
    pass

  def test_CalulcateOverlapTuple(self):
    location = PersonLocation(0, (100, 250, 175, 420), -1)

    self.assertAlmostEqual(location.CalculateOverlapTuple((100, 250, 170, 75)),
                           1.0)
    self.assertAlmostEqual(location.CalculateOverlapTuple((100, 20, 100, 30)),
                           0.0)

if __name__ == '__main__':
  unittest.main()
