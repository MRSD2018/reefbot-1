#!/usr/bin/python
'''Plots graphs of the labeling results.'''
usage='PlotLabelingResults.py [options]'

from pylab import *
import numpy as np
from optparse import OptionParser
import re
import os.path
import os
import ImageSearchResults

def CalculateStraightAccuracy(correctLabels, humanLabels):
  return np.sum(correctLabels) / float(len(humanLabels))

def CalculateMeanErrorPerClass(correctLabels, humanLabels,
                               minEntriesPerClass=15):
  sumError = 0.0
  nValidClasses = 0.0
  classes = np.unique(humanLabels)
  for classLabel in classes:
    validEntries = np.nonzero(humanLabels == classLabel)
    nEntries = float(len(validEntries[0]))
    if nEntries < minEntriesPerClass:
      continue

    sumError = sumError + np.sum(correctLabels[validEntries]) / nEntries
    nValidClasses = nValidClasses + 1

    #print 'Error for label %s is: %f' % (classLabel, np.sum(correctLabels[validEntries]) / nEntries)

  return sumError / nValidClasses

def CalculateAccuracies(mixingResults, accuracyFunc):
  '''Calculates the cumulative accuracies for the top labels averaged out over the different runs.

  Inputs:
  result from ParseMixingResults

  Returns: Tuple of ([mixing%], [[accuracyFor1stChoice], [accuracyFor2ndChoice]...]
  '''
  accuracies = []
  
  # Get the set of mixing percentages
  mixingParams = set([x[1] for x in mixingResults.keys()])
  mixingParams = [x for x in mixingParams]
  mixingParams.sort()

  accuracies = []
  curIdx = 0
  for mixingParam in mixingParams:
    accuracies.append([])
    # Iterate through all the different video ids
    for (key, data) in filter(lambda x: x[0][1] == mixingParam,
                              mixingResults.items()):
      if len(accuracies[curIdx]) == 0:
        accuracies[curIdx] = [[] for x in range(len(data[2]))]

      # Build a boolean matrix of which entries match the human label
      correctLabels = []
      for i in range(len(data[2])):
        correctLabels.append(data[1] == data[2][i][0])
      correctLabels = np.transpose(np.vstack(correctLabels))

      # Now turn it into a matrix specifying whether the correct
      # answer was in this guess or any before it.
      correctLabels = np.cumsum(correctLabels, axis=1)
      correctLabels = correctLabels > 0

      # Finally load up the accuracies
      for i in range(len(data[2])):
        accuracies[curIdx][i].append(accuracyFunc(correctLabels[:,i], data[1]))

    for i in range(len(accuracies[curIdx])):
      accuracies[curIdx][i] = np.mean(accuracies[curIdx][i])

    curIdx = curIdx + 1

  return (np.array(mixingParams), np.array(accuracies))

def PlotAccuracies(mixingParams, accuracies, xLabel='Fraction of weight using shape'):
  figure(1)
  plot(mixingParams, accuracies)
  xlabel(xLabel)
  ylabel('Accuracy')
  legend(('1st choice', '2nd choice', '3rd choice', '4th choice', '5th choice'))

  show()

def PlotDifferentTechniques(techniqueResults, legendStrings=None):
  figure(2)
  plot(range(1,6), np.transpose(np.vstack([x[1] for x in techniqueResults])))
  xlabel('Correct match found in the Nth result')
  ylabel('Accuracy')
  if legendStrings is None:
    legend([x[0] for x in techniqueResults], loc=4)
  else:
    legend(legendStrings, loc=4)
  show()

def PlotAccuraciesOfDifferentTechniques(mixingParams, techniqueResults,
                                        legendStrings=None, xLabel=''):
  figure(3)
  if legendStrings is None:
    legendStrings = [x[0] for x in techniqueResults]
    
  markers = ['o', 'x', 'v', '*', 'D', '^']
  for i in range(len(techniqueResults)):
    plot(mixingParams, techniqueResults[i][1][:,0], marker=markers[i],
         label=legendStrings[i], hold=True)

  plot([0.0, 8.0], [1./17, 1./17], label='Random', linewidth=3.0)
  xlabel(xLabel)
  ylabel('Mean Per Class Accuracy')
  legend()
  show()
                                        
      

if __name__ == '__main__': 
  # All of the command line flags
  parser = OptionParser(usage=usage)

  parser.add_option('--experiment_dir', dest='experiment_dir',
                    help='Directory containing the results of the experiment',
                    default='.')
  parser.add_option('--video_regexp', dest='video_regexp',
                    help='Regular expression used to extract the video id from the filename',
                    default='video([0-9]+)')
  parser.add_option('--shape_weight_regexp', dest='shape_weight_regexp',
                    help='Regular expression used to extract the shape weight from the filename',
                    default='shw([0-9\.]+)')
  parser.add_option('--mixing_file_regexp', dest='mixing_file_regexp',
                    help='Regular expression used to determine which files contain data about the mixing experiment',
                    default='results_shw[0-9\.]+_video[0-9]+_blob[0-9]+\.csv')
  parser.add_option('--xlabel', dest='xlabel',
                    help='X axis label of the graph',
                    default='Fraction of weight using shape')
  parser.add_option('--legend', dest='legend',
                    help='Comma separated list of legend strings',
                    default=None)
  parser.add_option('--normal_accuracy', dest='normal_accuracy',
                    action="store_true", default=False)
  parser.add_option('--class_accuracy', dest='class_accuracy',
                    action="store_true", default=False)
  parser.add_option('--min_class_size', default=15,
                    help='Minimum number of entries for a class label in order to include it')


  (options, args) = parser.parse_args()

  # Check to see if there are multiple experiment dirs separated by
  # semi-colons. If so, we're doing a plot where the legend is those
  # directories.
  experimentDirs = options.experiment_dir.split(',')

  # Figure out which type of accuracy to use
  accuracyFunc = None
  if options.normal_accuracy:
    accuracyFunc = CalculateStraightAccuracy
  elif options.class_accuracy:
    accuracyFunc = lambda x, y: CalculateMeanErrorPerClass(x, y, options.min_class_size)

  if options.legend is None:
    legendStrings = None
  else:
    legendStrings = options.legend.split(',')
  
  techniqueResults = []
  for experimentDir in experimentDirs:

    mixingFiles = ImageSearchResults.GetMixingFiles(experimentDir,
                                                    re.compile(options.video_regexp),
                                                    re.compile(options.shape_weight_regexp),
                                                    re.compile(options.mixing_file_regexp))

    results = ImageSearchResults.ParseMixingResults(experimentDir,
                                                    mixingFiles)
    (mixingParams, accuracies) = CalculateAccuracies(results, accuracyFunc)

    if len(experimentDirs) == 1:
      PlotAccuracies(mixingParams, accuracies, xLabel=options.xlabel)
    else:
      techniqueResults.append((os.path.basename(experimentDir), accuracies))

  if len(techniqueResults) > 0:
    PlotAccuraciesOfDifferentTechniques(mixingParams, techniqueResults,
                                        legendStrings=legendStrings,
                                        xLabel=options.xlabel)
    #PlotDifferentTechniques(techniqueResults, legendStrings=legendStrings)
                      
  
