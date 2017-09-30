#!/usr/bin/python
'''Plots graphs of the labeling results.'''
usage='ShowMatchingImages.py [options]'

from pylab import *
import numpy as np
import csv
from optparse import OptionParser
import re
import os.path
import os
import ImageSearchResults

def ShowRetreivedImages(frameDir, results, indicies):  
  fig = figure(2)
  nReturns = len(results[2])
  curPlot = 1
  for i in indicies:
    ax = fig.add_subplot(len(indicies), nReturns + 1, curPlot)
    curPlot = curPlot + 1
    
    imshow(imread(GetFrameFilename(frameDir, results[0][i])),
           origin='lower')
    ax.xaxis.set_ticks([])
    ax.yaxis.set_ticks([])
    

    for j in range(nReturns):
      ax = fig.add_subplot(len(indicies), nReturns+1, curPlot)
      curPlot = curPlot + 1

      if results[2][j][0][i] == -1:
        continue
      imshow(imread(os.path.join(frameDir, results[2][j][2][i] + '.jpg')),
              origin='lower')
      if results[1][i] == results[2][j][0][i]:
        color = 'r'
      else:
        color='k'
        
      title('%5.3f' % results[2][j][1][i], color=color)
      ax.xaxis.set_ticks([])
      ax.yaxis.set_ticks([])

  show()

def GetFrameFilename(frameDir, pair):
  return os.path.join(frameDir, '%s.%i.jpg' % pair)
      

if __name__ == '__main__': 
  # All of the command line flags
  parser = OptionParser(usage=usage)

  parser.add_option('-i', dest='experiment_file',
                    help='File of an experiment result',
                    default=None)
  parser.add_option('--frame_dir', dest='frame_dir',
                    help='Directory containing the image frames',
                    default='.')
  parser.add_option('--min_index_show', type="int", default=0,
                    help="Minimim index in the results file to show examples from")
  parser.add_option('--max_index_show', type="int", default=1,
                    help="Maximum index in the results file to show examples from")

  (options, args) = parser.parse_args()

  result = ImageSearchResults.ParseResultsFile(options.experiment_file)

  ShowRetreivedImages(options.frame_dir, result,
                      range(options.min_index_show, options.max_index_show))
                      
  
