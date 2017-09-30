#!/usr/bin/python
'''Plots the histogram of the size of blobs in a histogram of blob files.'''
usage='PlotSizeHistogrampy [options]'

from pylab import *
import numpy as np
import Blob
from optparse import OptionParser
import os.path

def GetBlobSizes(blobList, blobDir):
  serializer = Blob.BlobSerializer()
  blobSizes = []
  for line in open(blobList):
    blobFile = open(os.path.join(blobDir, line.strip()))
    try:
      blobResult, frameImage = serializer.Deserialize(blobFile,
                                                      blobDir)
      for blob in blobResult:
        blobSizes.append(blob.Area())
    finally:
      blobFile.close()

  return blobSizes

def CalcHistogram(sizes, range, nBins):
  return hist(np.log10(np.array(sizes)), bins=nBins,
              range=range)

def AddHistogram(sizes, histResult, label, color):
  nBlobs = len(sizes)
  n, bins, patches = histResult
  n = np.array(n, dtype=np.float32)
  bar((bins[0:options.bins]+bins[1:])/2, n/nBlobs, width=(bins[1]-bins[0]),
      label=label, color=color)
  #plot((bins[0:options.bins]+bins[1:])/2,
  #     n/nBlobs,
  #     drawstyle='steps-mid', label=label,
  #     fillstyle='bottom')
  

if __name__ == '__main__':
  # All of the command line flags
  parser = OptionParser(usage=usage)

  parser.add_option('--human_blob_list',
                    help='Filename of blobs, one per line',
                    default=None)
  parser.add_option('--human_blob_dir',
                    help='Directory where the blob files reside',
                    default=None)
  parser.add_option('--machine_blob_list',
                    help='Filename of blobs, one per line',
                    default=None)
  parser.add_option('--machine_blob_dir',
                    help='Directory where the blob files reside',
                    default=None)
  parser.add_option('--range',
                    help='Python code to calculate the range of the histogram',
                    default='None')
  parser.add_option('--bins', type='int',
                    help='Number of bins in the histogram',
                    default=100)
  
  (options, args) = parser.parse_args()

  humanBlobSizes = GetBlobSizes(options.human_blob_list,
                                options.human_blob_dir)

  humanHist = CalcHistogram(humanBlobSizes, eval(options.range),
                            options.bins)

  machineBlobSizes = GetBlobSizes(options.machine_blob_list,
                                  options.machine_blob_dir)

  machineHist = CalcHistogram(machineBlobSizes, eval(options.range),
                              options.bins)

  figure(2)
  AddHistogram(humanBlobSizes, humanHist, 'Human', 'blue')
  AddHistogram(machineBlobSizes, machineHist, 'Automatic', 'green')
  

  xlabel('Size of detected patches (log(pixels))')
  ylabel('Fraction of fish instances found')
  legend()

  show()
