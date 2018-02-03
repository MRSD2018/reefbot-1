#!/usr/bin/python
'''Creates plots for how likely the blobs are to be fish.'''
usage='PlotFishLocalizationResults.py <mturkCSV>'

import roslib; roslib.load_manifest('species_id')
from pylab import *
import numpy as np
from optparse import OptionParser
import csv
import math
import re
import os.path

def CreateColMap(header):
  colMap = {}
  for entry in header:
    colMap[entry] = header.index(entry)
  return colMap

def ParseResults(reader, colMap):
  imgList = []
  for entry in reader:

    # If the HIT was rejected, ignore it
    if entry[colMap['AssignmentStatus']] == 'Rejected':
      continue

    # Parse the result and keep those that were labeled as being
    # fish. Non-fish are labeled as -1
    resultString = entry[colMap['Answer.image_labels']]
    for answer in resultString.split(';'):
      sp = answer.split(':')
      if len(sp) <> 2:
        continue
      filename = sp[0];
      resultClusterId = int(sp[1])
      imgList.append((filename, resultClusterId))

  return imgList

class StatExtractor:
  def __init__(self):
    pass

  def ExtractStat(self, blob):
    raise NotImplementedError()

class DistanceFromCenter(StatExtractor):
  def __init__(self):
    StatExtractor.__init__(self)

  def ExtractStat(self, blob):
    box = blob.GetBoundingBox()
    x = (box[0] + box[2]) / 2.0
    y = (box[1] + box[3]) / 2.0
    return math.sqrt(math.pow(540-x, 2) + math.pow(960-y,2))

class BlobArea(StatExtractor):
  def __init__(self):
    StatExtractor.__init__(self)

  def ExtractStat(self, blob):
    return blob.Area()

class Eccentricity(StatExtractor):
  def __init__(self):
    StatExtractor.__init__(self)

  def ExtractStat(self, blob):
    box = blob.GetBoundingBox()
    x = double(box[2] - box[0])
    y = double(box[3] + box[1])
    if x > y:
      return y / x
    else:
      return x / y

def CollectStats(imgList, statExtractor, blobDir, blobIdRegexp, blobRegexp):
  '''Creates an array of a given stat for the image list.

  Inputs:
  imgList - [(filename, clusterId)]
  statExtractor - StatExtractor object for the stat to extract
  blobDir - Directory that contains the blob files

  Output:
  dictionary mapping cluterId -> [stats]
  * The stats are one per imgList and the order is preserved
  '''
  retval = {}
  for filename, clusterId in imgList:
    if not clusterId in retval.keys():
      retval[clusterId] = []

    # Open up the blob
    blobId = int(blobIdRegexp.search(filename).groups()[0])
    blobFilename = blobRegexp.search(filename).groups()[0]
    blobFile = open(os.path.join(blobDir, blobFilename))
    try:
      blobResult, imgName = Blob.BlobSerializer().Deserialize(blobFile, blobDir)
    finally:
      blobFile.close()
    blob = blobResult.GetBlob(blobId)

    retval[clusterId].append(statExtractor.ExtractStat(blob))

  return retval
                           

def DrawScatterGraph(titleStr, imgList, statExtractorX, statExtractorY,
                     blobDir, blobIdRegexp, blobRegexp):
  xStats = CollectStats(imgList, statExtractorX, blobDir, blobIdRegexp,
                        blobRegexp)
  yStats = CollectStats(imgList, statExtractorY, blobDir, blobIdRegexp,
                        blobRegexp)
  figure()
  for cluster, marker in [(-1,'o'), (0,'+')]:
    scatter(xStats[cluster], yStats[cluster], marker=marker)
    hold(True)

  title(titleStr)
  hold(False)
  show()

def DrawHistograms(titleStr, imgList, statExtractor, blobDir, blobIdRegexp,
                   blobRegexp):
  stats = CollectStats(imgList, statExtractor, blobDir, blobIdRegexp,
                       blobRegexp)

  figure()

  maxVal = max(max(stats[-1]), max(stats[0]))
  minVal = min(min(stats[-1]), min(stats[0]))

  for cluster, label, color in [(-1,'false', 'r'), (0,'true', 'b')]:
    hist(stats[cluster], bins=50, range=(minVal, maxVal),
         histtype='step', label=label, color=color)
    hold(True)

  title(titleStr)
  legend()
  hold(False)
  show()

if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('--blob_dir', 
                    help='Directory of the blob files',
                    default='.')
  parser.add_option('--blobid_regexp', dest='blobid_regexp',
                    help='Regular expression used to extract the blob id from the filename',
                    default='blob\.([0-9]+)\.')
  parser.add_option('--blob_regexp', dest='blob_regexp',
                    help='Regular expression used to extract the blob filename from the image filename',
                    default='(.+\.blob)')


  (options, args) = parser.parse_args()
 
 csvFile = args[-1]

 reader = csv.reader(open(csvFile, 'r'))
  colMap = CreateColMap(reader.next())
  imgList = ParseResults(reader, colMap)

  blobRegexp = re.compile(options.blob_regexp)
  blobIdRegexp = re.compile(options.blobid_regexp)

  #DrawHistograms('Distance to Center', imgList, DistanceFromCenter(),
  #               options.blob_dir, blobIdRegexp, blobRegexp)
  DrawHistograms('Area', imgList, BlobArea(),
                 options.blob_dir, blobIdRegexp, blobRegexp)
  DrawHistograms('Eccentricity', imgList, Eccentricity(),
                 options.blob_dir, blobIdRegexp, blobRegexp)

  show()
