#!/usr/bin/python
'''Creates a set of clustering HITS for all the unlabeled images.

Uses a labeling file of the form:
<image>,<speciesId>

As the known labels. And a file with one filename per line as the possible
images.'''
usage='CreateClusterHITSForUnlabeledImages.py <knownFile> <imageList> <outputFile>'

import os.path
from optparse import OptionParser

def LoadKnownLabels(knownFile):
  knownLabels = []
  for line in open(knownFile):
    knownLabels.append(line.strip().split(',')[0])

  return set(knownLabels)

def CreateHITS(imageList, maxImages=16, prefix=''):
  # Now create the string for these hits
  curIndex = 0;
  retString = ''
  while curIndex < len(imageList):
    # Specify the prefix on the line first
    retString = retString + prefix + ','

    # Now write the image filenames for this HIT
    lastIndex = curIndex + maxImages
    if lastIndex > len(imageList):
      lastIndex = len(imageList)
    retString = retString + ';'.join(imageList[curIndex:lastIndex]) + '\n'

    curIndex = curIndex + maxImages

  return retString
                       

if __name__ == '__main__':
  # All of the command line flags
  parser = OptionParser(usage=usage)

  parser.add_option('--prefix', default=None,
                    help='The prefix for the filename')
  parser.add_option('--max_images', 
                    help='Maximum number of images that can be in a particular HIT',
                    default=16, type='int')

  (options, args) = parser.parse_args()
  
  knownFile = args[0]
  imageListFn = args[1]
  outputFile = args[2]

  if options.prefix is not None:
    prefix = options.prefix

  knownLabels = LoadKnownLabels(knownFile)

  imageList = []
  for line in open(imageListFn):
    imageName = line.strip()
    if imageName not in knownLabels:
      imageList.append(imageName)
  

  outputStream = open(outputFile, 'w')
  try:
    outputStream.write("PREFIX,IMAGESTR\n")
    outputStream.write(CreateHITS(imageList, options.max_images, prefix))
  finally:
    outputStream.close()
