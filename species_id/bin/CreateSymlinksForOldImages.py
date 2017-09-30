#!/usr/bin/python
'''Creates symlinks from images with old frame ids, to new ones.

Each symlink will be named like the old file and point to the new one. For this to work, we use three directories:

oldDir - Directory containing all the old named images
newDir - Directory containing the new linked images
symDir - Directory to create the new symlinks in.

It requires that images are equal using a file diff.
'''
usage='CreateSymlinksForOldImages.py <oldDir> <newDir> <symDir>'

import os
import os.path
from optparse import OptionParser
import re
import filecmp

def FindImagesInDir(directory, frameRe):
  '''Finds all the images in a directory that match the frame regex and return a dictionary of image lists, one for each movie name.'''
  sortFunc = lambda x: frameRe.search(x).groups()[0]
  
  imagesFound = (x for x in os.listdir(directory) if frameRe.search(x))

  retval = {}
  for imageName in imagesFound:
    movieName = frameRe.search(imageName).groups()[1]
    if movieName not in retval:
      retval[movieName] = []
    retval[movieName].append(imageName)

  for key, imageList in retval.iteritems():
    retval[key] = sorted(retval[key], key=sortFunc)

  return retval

if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('--frame_regex',
                    default='((([0-9][0-9]-){3})[0-9]+)\.',
                    help='Regex to extract the frame id from filenames')

  (options, args) = parser.parse_args()

  oldDir = args[0]
  newDir = args[1]
  symDir = args[2]

  frameRe = re.compile(options.frame_regex)

  # Create the output directory if necessary
  if not os.path.exists(symDir):
    os.makedirs(symDir)

  
  oldImages = FindImagesInDir(oldDir, frameRe)
  newImages = FindImagesInDir(newDir, frameRe)

  for movieName, oldImageList in oldImages.iteritems():
    if movieName not in newImages.keys():
      print 'ERROR: Could not find new images for movie: ' + movieName
      continue
    for oldImageName in oldImageList:
      foundMatch = False
      for newImageName in newImages[movieName]:
        if filecmp.cmp(os.path.join(oldDir, oldImageName),
                       os.path.join(newDir, newImageName)):
          foundMatch = True
          symFile = os.path.join(symDir, oldImageName)
          if os.path.exists(symFile) and os.path.islink(symFile):
            os.remove(symFile)
          os.symlink(os.path.join(newDir, newImageName),
                       symFile)

      if not foundMatch:
        print 'WARN: Could not find a matching image for: ' + oldImageName
