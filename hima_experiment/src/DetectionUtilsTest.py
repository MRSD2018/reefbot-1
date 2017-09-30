#!/usr/bin/python

from DetectionUtils import *
import copy
import unittest

class NMSTest(unittest.TestCase):
  def setUp(self):
    pass

  def assertScoresEqual(self, orig, filtered):
    self.assertEqual(len(orig), len(filtered))
    for i in range(len(orig)):
      self.assertEqual(orig[i][0], filtered[i][0])
      self.assertAlmostEqual(orig[i][1], filtered[i][1])

  def assertOrderSame(self, orig, filtered):
    curIdx = 0
    for entry in filtered:
      while orig[curIdx][0] <> entry[0]:
        curIdx += 1

      if curIdx >= len(orig):
        self.fail("The order is broken. Could not find %s" % (entry))

  def test_NoOverlap(self):
    orig = [((4, 5, 3, 2), 0.5),
            ((10, 10, 8, 8), 1.0)]

    filtered = ApplyNMS(copy.deepcopy(orig), -1, 0)

    self.assertScoresEqual(orig, filtered)

  def test_NoScores(self):
    filtered = ApplyNMS([], -1, 0)

    self.assertScoresEqual(filtered, [])

  def test_NotEnoughOverlap(self):
    orig = [((4, 5, 5, 2), 0.5),
            ((4, 5, 2, 2), 1.0)]

    filtered = ApplyNMS(copy.deepcopy(orig), -1, 0)

    self.assertScoresEqual(orig, filtered)

  def test_JustEnoughOverlapRemove(self):
    orig = [((4, 5, 3, 2), 1.0),
            ((4, 5, 5, 2), 0.5),
            ((10, 10, 8, 8), 1.5)]

    filtered = ApplyNMS(copy.deepcopy(orig), None, 0)

    self.assertOrderSame(orig, filtered)
    self.assertAlmostEqual(filtered[0][1], 1.0)
    self.assertAlmostEqual(filtered[1][1], 1.5)

  def test_JustEnoughOverlapRemove2(self):
    orig = [((4, 5, 5, 2), 0.5),
            ((4, 5, 3, 2), 1.0),
            ((10, 10, 8, 8), 1.5)]

    filtered = ApplyNMS(copy.deepcopy(orig), None, 0)

    self.assertOrderSame(orig, filtered)
    self.assertAlmostEqual(filtered[0][1], 1.0)
    self.assertAlmostEqual(filtered[1][1], 1.5)

  def test_JustEnoughOverlapRevalue(self):
    orig = [((4, 5, 5, 2), 0.5),
            ((4, 5, 3, 2), 1.0),
            ((10, 10, 8, 8), 1.5)]

    filtered = ApplyNMS(copy.deepcopy(orig), -1.0, 0)

    self.assertOrderSame(orig, filtered)
    self.assertAlmostEqual(filtered[0][1], -1.0)
    self.assertAlmostEqual(filtered[1][1], 1.0)
    self.assertAlmostEqual(filtered[2][1], 1.5)

  def test_TwoSetsOfOverlaps(self):
    orig = [((4, 5, 3, 2), 0.5),
            ((4, 5, 5, 2), 1.0),
            ((4, 6, 5, 2), 0.7),
            ((0, 0, 3, 4), 0.1),
            ((0, 0, 4, 5), 0.2)]
    
    filtered = ApplyNMS(copy.deepcopy(orig), -1.0, 0)
    self.assertOrderSame(orig, filtered)
    self.assertAlmostEqual(filtered[0][1], -1.0)
    self.assertAlmostEqual(filtered[1][1], 1.0)
    self.assertAlmostEqual(filtered[2][1], -1.0)
    self.assertAlmostEqual(filtered[3][1], -1.0)
    self.assertAlmostEqual(filtered[4][1], 0.2)

  def test_SomeEntriesNotChosen(self):
    orig = [((4, 5, 5, 2), 0.5),
            ((4, 5, 3, 2), 1.0),
            ((10, 10, 8, 8), 1.5)]

    filtered = ApplyNMS(copy.deepcopy(orig), -1.0, 0.7)
    
    self.assertScoresEqual(orig, filtered)

if __name__ == '__main__':
  unittest.main()
