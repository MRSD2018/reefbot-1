#!/usr/bin/python
'''Plots the PR curve of fish extraction.'''
usage='PloBoxPRCurve.py [options]'

from pylab import *
import numpy as np
from optparse import OptionParser
import re
import os.path
import os
import xml2blob
import Blob

if __name__ == '__main__':
  # All of the command line flags
  parser = OptionParser(usage=usage)

  parser.add_option('--frame_regex',
                    default='(([0-9][0-9]-){3}[0-9]+)\.',
                    help='Regex to extract the frame id from filenames')
  parser.add_option('--machine_blob_dir', default=None,
                    help='Directory that contains the machine blobs blobs')
  parser.add_option('--human_blob_dir', default=None,
                    help='Directory that contains the human blobs')
  parser.add_option('--machine_labels', default=None,
                    help='File containing the known labels for the blobs in known_blob_dir. Each line is <filename>,<label>')
  parser.add_option('--blob_regex',
                    default='blob\.([0-9]+)\.',
                    help='Regex to extract the blob id from a filename')
  parser.add_option('--min_overlap', default=10, type='int',
                    help='Minimum number of pixels overlap to transfer the label')

  (options, args) = parser.parse_args()

  frameRegex = re.compile(options.frame_regex)
  blobRegex = re.compile(options.blob_regex)
  blobSerializer = Blob.BlobSerializer()

  # Load in the known labels
  machineLabels = {}
  for line in open(options.machine_labels):
    f, label = line.strip().split(',')
    frameId = frameRegex.search(f).groups()[0]
    blobId = int(blobRegex.search(f).groups()[0])
    machineLabels[(frameId, blobId)] = label

  # Open up the human blobs one at a time and start counting
  blobFileRegex = re.compile('.*\.blob')
  NO_LABEL = -1
  imgShape = (544, 960)
  missedFish = 0
  blobSizes  = []
  labels = []
  for filename in os.listdir(options.human_blob_dir):
    if not blobFileRegex.match(filename):
      continue

    frameId = frameRegex.search(filename).groups()[0]

    humanBlobs = blobSerializer.Deserialize(open(os.path.join(
      options.human_blob_dir, filename)),
                                            options.human_blob_dir)[0]
    machineBlobs = blobSerializer.Deserialize(open(os.path.join(
      options.machine_blob_dir,  filename)),
                                              options.machine_blob_dir)[0]

    # Count the number of fish found by humans but not by machine
    for humanBlob in humanBlobs:
      foundMatch = False
      curHumanBlob = humanBlob.ToBinaryImageMask(imgShape)
      for machineBlob in machineBlobs:
        curMachineBlob = machineBlob.ToBinaryImageMask(imgShape)

        overlap = np.sum(np.bitwise_and(curMachineBlob, curHumanBlob))
        if overlap > options.min_overlap:
          foundMatch = True
          break

      if not foundMatch:
        missedFish = missedFish + 1

    # Get the labels and sizses for all the machine blobs
    for i in range(machineBlobs.nBlobs()):
      label = NO_LABEL
      try:
        label = int(machineLabels[(frameId, i)])
      except KeyError: pass

      labels.append(label)
      blobSizes.append(machineBlobs.GetBlob(i).Area())

  # Now that we have the labels, calculate the precision and recall curves
  precision = []
  recall = []

  sizeThresholds = sorted(set(blobSizes))
  sizeThresholds.reverse()
  labels = np.array(labels)
  blobSizes = np.array(blobSizes)
  for size in sizeThresholds:
    validIndex = np.nonzero(blobSizes >= size)
    truePos = float(np.sum(labels[validIndex] > 3))
    falsePos = float(np.sum(labels[validIndex] <= 3))
    falseNeg = float(np.sum(labels > 3) - truePos + missedFish)

    precision.append(truePos / (truePos + falsePos))
    recall.append(truePos / (truePos + falseNeg))

  figure(1)
  plot(np.array(recall), np.array(precision))
  xlabel('Recall')
  ylabel('Precision')
  axis([0, 0.7, 0, 1.0])

  show()
  
  
