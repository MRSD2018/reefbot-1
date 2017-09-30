#!/usr/bin/python
'''Plots the PR curve of the labeling results.'''
usage='PlotLabelingPRCurve.py [options]'

from pylab import *
import numpy as np
from optparse import OptionParser
import re
import os.path
import os
import ImageSearchResults

def GetResultFiles(directory, fileRegex):
  '''Returns a list of files to process.'''
  retVal = []
  for filename in os.listdir(directory):
    if fileRegexp.match(filename):
      retVal.append(filename)
  return retVal

def CalculatePRCurve(results, recallPoints):
  humanLabels = np.array(results[1], dtype=np.int32)
  machineLabels = np.array(results[2][0][0], dtype=np.int32)
  machineScores = results[2][0][1]

  uniqueScores = sorted(set(machineScores))
  uniqueScores.reverse()
  machineScores = np.array(machineScores, dtype='float32')

  nEntries = humanLabels.shape[0]
  precision = []
  recall = []
  for score in uniqueScores:
    validIdx = np.nonzero(machineScores >= score)
    nAnswers = len(validIdx[0])
    nCorrect = np.sum(humanLabels[validIdx] == machineLabels[validIdx])
    precision.append(float(nCorrect) / float(nAnswers))
    recall.append(float(nAnswers) / float(nEntries))

  return np.interp(recallPoints, np.array(recall, dtype=np.float32),
                   np.array(precision, dtype=np.float32), right=0.0)

def ProcessDirectory(directory, fileRegex, recallPoints):
  precision = []
  for f in GetResultFiles(directory, fileRegex):
    results = ImageSearchResults.ParseResultsFile(os.path.join(directory,f))

    precision.append(CalculatePRCurve(results, recallPoints))

  precision = np.array(precision, dtype=np.float32)
  return np.mean(precision, axis=0)
  

if __name__ == '__main__': 
  # All of the command line flags
  parser = OptionParser(usage=usage)

  parser.add_option('--input', '-i',
                    help="File of <directory>,<name> of inputs to use",
                    default=None)
  parser.add_option('--random_line', type='float',
                    help='Value of a random classifier',
                    default=0.1)
  parser.add_option('--file_regex',
                    help='Regexp to select the filenames in each directory',
                    default='.*shw1.0.*blob20.*\.csv')
  parser.add_option('--axis',
                    help='Python code specifying the axis to display',
                    default=None)

  (options, args) = parser.parse_args()

  markers = ['o', 'x', 'v', '*', 'D', '^']

  fileRegexp = re.compile(options.file_regex)

  dirs = []
  for line in open(options.input):
    directory, displayName = line.strip().split(',')
    dirs.append((directory, displayName))

  recallPoints = np.arange(0.0, 1.0, 0.005)
  curves = []
  for directory, displayName in dirs:
    curves.append(ProcessDirectory(directory, fileRegexp, recallPoints))

  figure(1)
  for i in range(len(curves)):
    plot(recallPoints, np.array(curves[i], dtype=np.float32), hold=True,
         label=dirs[i][1], marker=markers[i])
  plot([0.0, 1.0], [options.random_line, options.random_line], hold=True,
       label='Random', linewidth=3)
  #legendVals = [x[1] for x in dirs]
  #legendVals.append('Random')
  legend()
  xlabel('Fraction of Queries With Responses')
  ylabel('Accuracy')
  if options.axis is not None:
    axis(eval(options.axis))

  show()
