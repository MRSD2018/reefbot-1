#! /usr/bin/python
'''A script that converts a video sequence in idl/frames to HIMA.'''
usage='EthToHima.py [options] <idl_file> <outputDir>'

from optparse import OptionParser
import os
import os.path
import re
import time

if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('--fps', type='float',
                    help='fps for the annotations in the video',
                    default=30.0)

  (options, args) = parser.parse_args()

  idlFile = args[0]
  outputDir = args[1]

  # Create the output directories
  if not os.path.exists(outputDir):
    os.makedirs(outputDir)
  imageDir = os.path.join(outputDir, 'left')
  if not os.path.exists(imageDir):
    os.makedirs(imageDir)

  # Open the annotation file for output
  annotFile = open(os.path.join(outputDir, 'annotations.txt'), 'w')

  # Regular expressions to extract data from each line in the idl file
  imgNameRe = re.compile(r'\"(left/image_[0-9_]+\.png)\"')
  coordRe = re.compile(r'\(([0-9, ]+)\)')

  # Read through each line of the input annotations and convert
  curFrameId = 0
  curPersonId = 0
  curTime = time.time()
  for line in open(idlFile):
    imgFile = imgNameRe.search(line).groups()[0]
    if not imgFile:
      raise 'Could not find the image file in %s' % line

    if not coordRe.search(line):
      # There are no annotations for this line
      continue

    # Create a symlink to the image
    newImgFile = os.path.join(imageDir, 'img_%04i.png' % curFrameId)
    if os.path.exists(newImgFile):
      os.remove(newImgFile)
    os.symlink(os.path.join(os.path.dirname(idlFile), imgFile),
               newImgFile)
    
    for coordStr in coordRe.finditer(line):
      coordSplit = coordStr.groups()[0].split(', ')
      x1 = int(coordSplit[0])
      y1 = int(coordSplit[1])
      x2 = int(coordSplit[2])
      y2 = int(coordSplit[3])

      annotFile.write('%f %i %i 1 %.2f %.2f %.2f %.2f -1 -1 -1 -1 %.2f %.2f -1 -1 -1 -1 -1\n' % (curTime, curFrameId, curPersonId, x1, y1, x2, y2,
         (x1+x2)/2.0, (y1+y2)/2.0))

      curPersonId += 1
      

    curFrameId += 1
    curTime += 1.0/options.fps                                     
