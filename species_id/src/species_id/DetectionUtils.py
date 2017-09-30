import roslib; roslib.load_manifest('species_id')
import rospy
import numpy as np
from sensor_msgs.msg import RegionOfInterest
import cDetectionUtils
import math

def CalculateOverlapOfRegions(tupA, tupB):
  return cDetectionUtils.CalculateOverlapOfRegions(tupA, tupB)

def OldCalculateOverlapOfRegions(tupA, tupB):
  '''Calculates the overlap between two tuples of the form (x, y, h, w).'''
  if min(tupA[2], tupA[3], tupB[2], tupB[3]) < 0:
    return 0.0
    
  midYA = tupA[1] + tupA[2]/2.0
  midYB = tupB[1] + tupB[2]/2.0
  dRy = tupA[2]/2.0 + tupB[2]/2.0 - abs(midYA - midYB)

  midXA = tupA[0] + tupA[3]/2.0
  midXB = tupB[0] + tupB[3]/2.0
  dRx = tupA[3]/2.0 + tupB[3]/2.0 - abs(midXA - midXB)

  if dRx < 1e-5 or dRy < 1e-5:
    # They do not overlap
    return 0.0

  # They overlap so calculate the fraction overlap as the area that
  # overlaps over the area of the union.
  areaIntersection = float(dRy * dRx)
  areaUnion = (float(tupA[2]*tupA[3]) +
               float(tupB[2]*tupB[3]) -
               areaIntersection)
  return areaIntersection / areaUnion

def CalculateOverlapWithGrid(tup, grid):
  '''Calculates the overlap with a grid.

  The grid defines a 4D region where grid[w,h,x,y,:] will be the tuple for that point on the grid.

  Returns a 4D grid with the overlap values.
  '''
  midYA = tup[1] + tup[2]/2.0
  midYB = grid[:,:,:,:,1] + grid[:,:,:,:,2]/2.0
  dRy = tup[2]/2.0 + grid[:,:,:,:,2]/2.0 - np.abs(midYA - midYB)
  dRy = np.maximum(dRy, 0)

  midXA = tup[0] + tup[3]/2.0
  midXB = grid[:,:,:,:,0] + grid[:,:,:,:,3]/2.0
  dRx = tup[3]/2.0 + grid[:,:,:,:,3]/2.0 - np.abs(midXA - midXB)
  dRx = np.maximum(dRx, 0)

  areaIntersection = dRy * dRx
  areaUnion = (float(tup[2]*tup[3]) + grid[:,:,:,:,2]*grid[:,:,:,:,3] -
               areaIntersection)

  return areaIntersection / areaUnion

  

def ApplyNMS(scores, newScore=None, chosenThresh=None, overlapThresh=0.5):
  '''Applies non maximal suppression.

  Inputs:
  scores: a list of (region, score) tuples to be filtered
  newScore: the value to replace the score of supressed regions with. If none, that region is removed
  chosenThresh: threshold for if a region is picked
  overlapThresh: the overlap between regions to supress the weaker one

  Outputs:
  A filtered version of scores, either the the scores of bad regions reduced, or removed entirely. Order is preserved.
  '''
  selectedRegions = []
  sortIdx = sorted(range(len(scores)), key=lambda x: scores[x][1],
                   reverse=True)
  for i in sortIdx:
    curRegion, curScore = scores[i]
    if chosenThresh is not None and curScore < chosenThresh:
      # All of the regions beyond this can't be chosen, so we're done
      break
    
    doesOverlap = False
    for selRegion in selectedRegions:
      if (cDetectionUtils.CalculateOverlapOfRegions(curRegion, selRegion) >
          overlapThresh):
        # This region overlaps with one that was already selected
        scores[i] = (curRegion, newScore)
        doesOverlap = True
        break

    if not doesOverlap:
      selectedRegions.append(curRegion)

  # Build the output if we are removing entries from the list
  if newScore is None:
    return [x for x in scores if x[1] is not None]

  return scores

def _CreateBox(x, y, height, width, doTuple=False):
  if doTuple:
    return (x, y, height, width)
  return RegionOfInterest(x, y, height, width, False)
    

def GenerateSamplingRegions(imgShape, gridMsg):
  '''Generates a dense grid of (x,y,h,w) tuples for a grid message.

  So, grid[i,j,k,m,:] will be the region for that point on the grid.
  '''
  assert(not gridMsg.fixAspect)
  
  heights = []
  curH = gridMsg.minH
  while curH < imgShape[0]-gridMsg.minY:
    heights.append(round(curH))
    curH *= gridMsg.strideH

  widths = []
  curW = gridMsg.minW
  while curW < imgShape[1]-gridMsg.minX:
    widths.append(round(curW))
    curW *= gridMsg.strideW

  if len(widths) == 0 or len(heights) == 0:
    return np.empty((0,0,0,0,0))

  grid = np.empty((
    len(widths),
    len(heights),
    max(1, (imgShape[1]-gridMsg.minX-widths[0])/gridMsg.strideX),
    max(1, (imgShape[0]-gridMsg.minY-heights[0])/gridMsg.strideY),
    4))

  for i in range(len(widths)):
    grid[i,:,:,:,3] = widths[i]

  for i in range(len(heights)):
    grid[:,i,:,:,2] = heights[i]

  for i in range(grid.shape[2]):
    grid[:,:,i,:,0] = gridMsg.minX + i*gridMsg.strideX

  for i in range(grid.shape[3]):
    grid[:,:,:,i,1] = gridMsg.minY + i*gridMsg.strideY
  
  return grid

def GetGridShape(gridMsg, imgShape):
  '''Returns the shape of a grid specified by the message. [w, h, x, y].'''
  return (int(math.log((imgShape[1]-gridMsg.minX) - math.log(gridMsg.minW)) /
              math.log(gridMsg.strideW)),
          int(math.log((imgShape[0]-gridMsg.minY) - math.log(gridMsg.minH)) /
              math.log(gridMsg.strideH)),
          int((imgShape[1]-gridMsg.minX-gridMsg.minW) / gridMsg.strideX),
          int((imgShape[0]-gridMsg.minY-gridMsg.minH) / gridMsg.strideY),
          

def GridIdx2Region(gridMsg, idx, imgShape=None, gridShape=None):
  '''Converts a flat index in a grid into a (x,y,h,w) tuple.'''
  if gridShape is None:
    gridShape = GetGridShape(gridMsg, imgShape)
  coords = np.unravel_index((idx), gridShape)
  
  return (gridMsg.minX + coords[2][0]*gridMsg.strideX,
          gridMsg.minY + coords[3][0]*gridMsg.strideY,
          round(gridMsg.minH * math.pow(gridMsg.strideH, coords[1][0])),
          round(gridMsg.minW * math.pow(gridMsg.strideW, coords[0][0])))
