#!/usr/bin/python
'''Creates blob files that contain rectangular regions with no fish in them.'''
usage='CreateNegativeExamples.py'

import roslib; roslib.load_manifest('species_id')
import rospy
import os
import os.path
from optparse import OptionParser
import re
from cv_blobs import Blob
import random
import cv2

def BoxesOverlap(boxA, boxB):
  if (boxA[0] > boxB[2] or
      boxB[0] > boxA[2] or
      boxA[1] > boxB[3] or
      boxB[1] > boxA[3]):
    return False
  return True

def GenerateNegativeBoxBlob(fishBlobs, minBoxSize, imgSize):
  '''Generates a random box that does not overlap any of the known fish.'''
  newBlob = None
  while newBlob is None:
    minX = random.randint(0, imgSize[0]-2)
    minY = random.randint(0, imgSize[1]-2)
    maxX = random.randint(minX, imgSize[0]-1)
    maxY = random.randint(minY, imgSize[1]-1)

    # See if the box is too small
    if (maxX-minX)*(maxY-minY) <= minBoxSize:
      continue

    candBox = (minX, minY, maxX, maxY)

    # See if the box overlaps with any of the bounding boxes around the fish
    overlapFish = False
    for fishBlob in fishBlobs:
      if BoxesOverlap(fishBlob.GetBoundingBox(), candBox):
        overlapFish = True
        break
    
    if not overlapFish:
      newBlob = Blob.Blob()
      newBlob.AddBox(minX, minY, maxX, maxY)

  return newBlob

if __name__ == '__main__':
  # All of the command line flags
  parser = OptionParser(usage=usage)

  parser.add_option('--prefix', default=None,
                    help='The prefix for the output blobs')
  parser.add_option('--nregions', 
                    help='Number of regions to find per image',
                    default=25, type='int')
  parser.add_option('--seed', 
                    help='Seed for the random number generator',
                    default=986725, type='int')
  parser.add_option('--frame_regex',
                    default='(([0-9][0-9]-){3}[0-9]+)\.',
                    help='Regex to extract the frame id from filenames')
  parser.add_option('-i', '--input', default=None,
                    help='File with blobs listed one per line')
  parser.add_option('--min_size', type='int', default=100,
                    help='Minimum blob size to accept in pixels')

  (options, args) = parser.parse_args()

  frameRe = re.compile(options.frame_regex)

  blobSerializer = Blob.BlobSerializer()

  prefix = options.prefix
  if prefix is None:
    prefix = ''
    
  negBlobDir = os.path.dirname(prefix)
  if not os.path.exists(negBlobDir):
    os.makedirs(negBlobDir)

  for line in open(options.input):
    filename = line.strip()
    foundFrame = frameRe.search(filename)
    if foundFrame:
      frameId = foundFrame.groups()[0]

      blobDir = os.path.dirname(filename)

      # Read the blobs that specify the fish
      blobStream = open(filename)
      try:
        fishBlobs, imgName = blobSerializer.Deserialize(blobStream, blobDir)
      finally:
        blobStream.close()

      # Find the bounds of the image
      cvImg = cv2.imread(imgName)
      if cvImg.shape[0] == 0 or cvImg.shape[1] == 0:
        rospy.error("Could not open image: " + imgName)
        continue

      # Build up some blobs
      random.seed(options.seed)
      negBlobs = Blob.BlobResult()
      while negBlobs.nBlobs() < options.nregions:
        negBlobs.AppendBlob(GenerateNegativeBoxBlob(fishBlobs,
                                                    options.min_size,
                                                    (cvImg.shape[1],
                                                     cvImg.shape[0])))

      # Write the blobs to a file
      negBlobFile = open('%s%s.blob' % (prefix, frameId), 'w')
      try:
        blobSerializer.Serialize(negBlobFile,
                                 negBlobs,
                                 os.path.relpath(os.path.join(blobDir,
                                                              imgName),
                                                 negBlobDir))
      finally:
        negBlobFile.close()
