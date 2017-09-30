#! /usr/bin/python
'''Converts a .blob file into a mask for where all the blobs are.'''
usage='MakeBlobMask.py [options]'

import roslib; roslib.load_manifest('species_id')
import rospy
import cv2
import Blob
import numpy as np
from optparse import OptionParser

if __name__ == '__main__':
  # All of the command line flags

  parser = OptionParser(usage=usage)

  parser.add_option('--input', '-i', dest='input',
                    help='Input blob file',
                    default=None)
  parser.add_option('--output', '-o', dest='output',
                    help='Output image file',
                    default=None)

  (options, args) = parser.parse_args()

  # Open the blob file
  blobSerializer = Blob.BlobSerializer()
  f = open(options.input)
  try:
    (blobs, imgName) = blobSerializer.Deserialize(f)
  finally:
    f.close()

  # Get the mask and convert it to a 255/0 binary file
  mask = blobs.ToBinaryImageMask() * 255

  # Grow the mask to be the same size as the image
  image = cv2.imread(imgName)
  bigMask = np.zeros(image.shape, np.uint8)
  bigMask[0:mask.shape[0], 0:mask.shape[1]] = mask
  #mask.resize((image.height, image.width))

  # Write the image out
  cv.SaveImage(options.output, cv.fromarray(bigMask))
