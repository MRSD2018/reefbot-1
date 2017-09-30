#!/usr/bin/python
import roslib; roslib.load_manifest('species_id')
import rospy
import unittest
from species_id.DetectionUtils import *
from objdetect_msgs.msg import Grid

class TestGridSampling(unittest.TestCase):

  def VerifyWholeGrid(self, grid, msg):
    # Grid should be [w,h,x,y,t] where t is the tuple. The tuple is
    # the form (x,y,h,w)
    self.assertEqual(len(grid.shape), 5)
    for w in xrange(grid.shape[0]):
      for h in xrange(grid.shape[1]):
        for x in xrange(grid.shape[2]):
          for y in xrange(grid.shape[3]):
            self.assertEqual(grid[w,h,x,y,0], msg.minX + x*msg.strideX)
            self.assertEqual(grid[w,h,x,y,1], msg.minY + y*msg.strideY)
            self.assertEqual(grid[w,h,x,y,2],
                             round(msg.minH*pow(msg.strideH, h)))
            self.assertEqual(grid[w,h,x,y,3],
                             round(msg.minW*pow(msg.strideW, w)))
    
  def test_emptyImage(self):
    msg = Grid(0, 0, 16, 16, 32, 32, 1.2, 1.2, False)
    regions = GenerateSamplingRegions((0,0), msg)
    self.VerifyWholeGrid(regions, msg)

  def test_normalImage(self):
    msg = Grid(0, 0, 16, 16, 32, 32, 1.2, 1.2, False)
    regions = GenerateSamplingRegions((200,400), msg)
    self.VerifyWholeGrid(regions, msg)

class TestOverlapMeasure(unittest.TestCase):
  def setUp(self):
    self.msg = Grid(0, 0, 16, 16, 32, 32, 1.2, 1.2, False)
    self.grid = GenerateSamplingRegions(
      (200,400),
      self.msg)

  def VerifyOverlaps(self, overlapTup):
    '''Verifies the overlaps in the grid by comparing with the the entry-by-entry approach.'''
    overlapGrid = CalculateOverlapWithGrid(overlapTup, self.grid)
    
    for w in xrange(self.grid.shape[0]):
      for h in xrange(self.grid.shape[1]):
        for x in xrange(self.grid.shape[2]):
          for y in xrange(self.grid.shape[3]):
            self.assertAlmostEqual(
              overlapGrid[w,h,x,y],
              CalculateOverlapOfRegions(overlapTup,
                                        self.grid[w,h,x,y,:]))
            
  def test_overlapEmpty(self):
    self.VerifyOverlaps((30, 30, 0, 0))

  def test_overlapPeopleSized(self):
    self.VerifyOverlaps((100, 50, 128, 64))
    

if __name__ == '__main__':
  unittest.main()
