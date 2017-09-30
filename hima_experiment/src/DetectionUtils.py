import roslib; roslib.load_manifest('hima_experiment')
import rospy
import numpy as np
from sensor_msgs.msg import RegionOfInterest
import cDetectionUtils

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
    

def GenerateSamplingRegions(cvImage, minRegionWidth, minRegionHeight,
                            winStride, scaleStride, doTuple=False):
  '''Generates a list of RegionOfInterest messages to evaluate around the image.'''
  regions = []
  curWidth = minRegionWidth
  curHeight = minRegionHeight
  curWinStride = winStride
  while curWidth < cvImage.shape[1] and curHeight < cvImage.shape[0]:
    for y_offset in np.arange(0, cvImage.shape[0] - curHeight, curWinStride):
      regions.extend([_CreateBox(int(round(x_offset)),
                                 int(round(y_offset)),
                                 int(round(curHeight)),
                                 int(round(curWidth)),
                                 doTuple) for
                      x_offset in np.arange(0, cvImage.shape[1] - curWidth,
                                            curWinStride)])

    curWidth = curWidth * scaleStride
    curHeight = curHeight * scaleStride
    curWinStride = curWinStride * scaleStride

  return regions
