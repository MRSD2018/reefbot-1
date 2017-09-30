#!/usr/bin/python
'''Plots the PR curves from the analysis created by the RunVUEstimatorPRAnalysis script.'''
usage='PlotVUEstimatorPRCurves.py [options] <inputFile0> <inputFile1> ...'

import roslib; roslib.load_manifest('species_id')
import rospy
from optparse import OptionParser
from pylab import *
import re
import os.path
import cPickle as pickle
import scipy.integrate
import numpy as np
from RunVUEstimatorPRAnalysis import *

def CalculateAveragePrecision(precision, recall, name):
  sortIdx = np.argsort(recall)
  curPrecision = precision[:,sortIdx]
  curRecall = recall[sortIdx]
  avgPrecision = scipy.integrate.trapz(curPrecision, curRecall, axis=1)
  rospy.loginfo('AP of %s: %f +- %f' %
                (name, np.mean(avgPrecision), np.std(avgPrecision)))  

if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('--output_prefix', default=None,
                    help='If the graphs should be saved to file, what is the prefix')

  parser.add_option('--group_names',
                    default=None,
                    help='Comma separated names for the legend in the graph')

  parser.add_option('--plot_step', type='int', default=1,
                    help='Step in the data when plotting to get a nicer plot')

  (options, args) = parser.parse_args()

  inputFiles = args

  # Start by figuring out the group names by using the filename if it is not known
  groupNames = []
  if options.group_names is None:
    stripExtRe = re.compile(r'(.*)\.prstats$')
    groupNames = [stripExtRe.match(os.path.basename(x)).groups()[0] for x in inputFiles]
  else:
    groupNames = options.group_names.strip().split(',')

  groupColors = ['b', 'g', 'r', 'k', 'm', 'y', '0.5']
  groupMarkers = ['o', '+', 'x', '^', 's', 'p', 'D']

  # Plot the precision recall curve
  fig = figure()
  legendLines = []
  for i in xrange(len(inputFiles)):
    curStats = pickle.load(open(os.path.abspath(inputFiles[i])))

    rospy.loginfo('Opening %s. Version is: %s' %
                  (inputFiles[i], curStats.VERSION))

    meanPrecision = np.mean(curStats.precision[:, 0::options.plot_step], 0)
    stdPrecision = np.std(curStats.precision[:, 0::options.plot_step], 0)

    # Plot the error regions
    fill_between(curStats.recall[0::options.plot_step],
                 meanPrecision - (stdPrecision / math.sqrt(curStats.precision.shape[0])),
                 meanPrecision + (stdPrecision / math.sqrt(curStats.precision.shape[0])),
                 color=groupColors[i], alpha=0.5)

    # Plot the mean line
    legendLines.append(semilogy(curStats.recall[0::options.plot_step],
                                meanPrecision,
                                c=groupColors[i],
                                marker=groupMarkers[i], hold=True))

    CalculateAveragePrecision(curStats.precision,
                              curStats.recall,
                              groupNames[i])

  xlabel('Recall')
  ylabel('Precision')
  legend(legendLines, groupNames)
  

  if options.output_prefix is not None:
    savefig(options.output_prefix + '_VuPrCurve.eps')
    savefig(options.output_prefix + '_VuPrCurve.png')

  show()
    
