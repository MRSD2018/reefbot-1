#!/usr/bin/python
'''Creates plots of performance vs. visaul utility accuray.

Uses a csv file where each line is <hogBag>,<vuBag0>,<vuBag1>,...
'''
usage='PlotVUAccuracy.py [options]'

import roslib; roslib.load_manifest('hima_experiment')
import rospy
import rosbag
import os
import os.path
import re
import gc
import glob
import re
import math
from pylab import *
from matplotlib.patches import Ellipse, Circle
import numpy as np
import scipy.interpolate
import scipy.integrate
import scipy.stats
from optparse import OptionParser
import cPickle as pickle
from VUAccuracy import VUAccuracyCalculator
from CascadeAccuracy import CascadeAccuracyStats
import PlottingUtils

import matplotlib
#matplotlib.rcParams.update({'font.family': 'serif'})

class TrainedStats:
  '''Class to hold the trained statistics.'''
  def __init__(self, regex, slowHogTime):
    self.reg = re.compile(regex)
    self.slowHogTime = slowHogTime
    self.statMap = {} # map from dataset->[stat objects]

  def AddStats(self, filename):
    dataset = self.reg.search(filename).groups()[0]
    if dataset not in self.statMap:
      self.statMap[dataset] = []

    self.statMap[dataset].append(pickle.load(open(filename, 'rb')))

  def GetPointMetric(self, xFunc, yFunc):
    '''Returns a tuple of lists
    (meanX, stdX, stdErrorX,meanY, stdY, stdErrorY)
    '''
    if len(self.statMap) == 0:
      return None
    xVals = []
    yVals = []
    for dataset, statObjects in self.statMap.iteritems():
      xVals.extend([xFunc(x) for x in statObjects])
      yVals.extend([yFunc(y) for y in statObjects])

    meanX = scipy.stats.nanmean(xVals)
    stdX = scipy.stats.nanstd(xVals)
    stdX = np.nan_to_num(stdX)
    stdErrorX = stdX / np.sqrt(np.sum(np.isfinite(xVals)))

    meanY = scipy.stats.nanmean(yVals)
    stdY = scipy.stats.nanstd(yVals)
    stdY = np.nan_to_num(stdY)
    stdErrorY = stdY / np.sqrt(np.sum(np.isfinite(yVals)))

    return (meanX, stdX, stdErrorX,
            meanY, stdY, stdErrorY)

  def GetMetric(self, xFunc, yFunc):
    '''Returns a tuple of lists (x, meanY, stdY, stdErrorY).'''
    if len(self.statMap) == 0:
      return None
    
    # Go through all the datasets and get the x values we care to
    # sample at about
    xVals = set()
    for dataset, statObjects in self.statMap.iteritems():
      xVals.update([xFunc(x) for x in statObjects])
    xVals = sorted(xVals)

    # Get samples of all the y values at the x values we care about,
    # interpolating if necessary.
    y = []
    for dataset, statObjects in self.statMap.iteritems():
      curX, curY = SortByX([xFunc(x) for x in statObjects],
                           [yFunc(x) for x in statObjects])
      f = scipy.interpolate.interp1d(
        curX,
        curY,
        kind='linear',
        bounds_error=False,
        fill_value=float('nan'))
      y.append(f(xVals))

    # Calculate the stats from the y values
    meanY = scipy.stats.nanmean(y, axis=0)
    stdY = scipy.stats.nanstd(y, axis=0)
    stdY = np.nan_to_num(stdY)
    stdErrorY = stdY / np.sqrt(np.sum(np.isfinite(y), axis=0))

    return (xVals, meanY, stdY, stdErrorY)

  def GetSpeedupVsMetric(self, yFunc):
    return self.GetMetric(
      lambda x: self.slowHogTime / x.meanProcessingTime[0], yFunc)

  def GetSpeedupVsPointMetric(self, yFunc):
    return self.GetPointMetric(
      lambda x: self.slowHogTime / x.meanProcessingTime[0], yFunc)

  def GetVuSpeedupVsMetric(self, yFunc):
    return self.GetMetric(
      lambda x: self.slowHogTime / x.meanVuProcessingTime[0], yFunc)

  def GetVuSpeedupVsPointMetric(self, yFunc):
    return self.GetPointMetric(
      lambda x: self.slowHogTime / x.meanVuProcessingTime[0], yFunc)
    

def SortByX(inX, inY, doSort=True):
  '''Sorts a line by the x values and fills in nans if necessary'''

  if doSort:
    sortIdx = sorted(range(len(inX)),
                     key=lambda x:
                     float('-inf') if math.isnan(inX[x]) else inX[x])
  else:
    sortIdx = range(len(inX))
  outX = np.array(inX)[sortIdx]
  outY = np.array(inY)[sortIdx]
  outY[np.logical_not(np.isfinite(outY))] = float('nan')
  return (outX, outY)

def PlotAccuracyScatter(baselineX, baselineY, accuracy, y, colors,
                        markers, groupNames, xLabel, yLabel, nVUEstimators,
                        logY=False, logX=False, doSort=True,
                        legendLoc=1,
                        validEntries=None,
                        trainedStats=None,
                        noHogTrainedStats=None,
                        ignoreTimeStats=None):

  plotFunc = plot
  if logY:
    plotFunc = semilogy
  elif logX:
    plotFunc = semilogx

  fig = figure()
  scatLines = []
  labels = []
  # Plot the baseline
  if baselineX is not None and len(baselineX) > 0:
    scatLines.append(plotFunc(baselineX, baselineY, marker='v', color='c',
                              hold=True))
    labels.append('Rescaling Image')

  # Plot the results
  for i in range(accuracy.shape[0]):
    for j in range(accuracy.shape[1]):
      if validEntries is not None and (j % nVUEstimators) not in validEntries:
        continue

      curX, curY = SortByX(accuracy[i][j], y[i][j], doSort=doSort)
      curLine = plotFunc(curX,
                         curY,
                         c=colors[j % nVUEstimators],
                         marker=markers[j % nVUEstimators],
                         hold=True,
                         scaley=False,
                         linewidth=2)
      if i == 0 and j < nVUEstimators:
        labels.append(groupNames[j])
        scatLines.append(curLine)

  # Plot the trained example
  if trainedStats is not None:
    scatLines.append(plotFunc(trainedStats[0],
                              trainedStats[1],
                              c='darkturquoise',
                              marker='<',
                              hold=True,
                              scaley=False,
                              linewidth=2))
    fill_between(trainedStats[0],
                 trainedStats[1] - trainedStats[3],
                 trainedStats[1] + trainedStats[3],
                 color='darkturquoise',
                 alpha=0.5)
    labels.append('SCATAT')

  # Plot the trained example that assumes no HOG detector
  if noHogTrainedStats is not None:
    scatLines.append(plotFunc(noHogTrainedStats[0],
                              noHogTrainedStats[1],
                              c='darkorchid',
                              marker='>',
                              hold=True,
                              scaley=False,
                              linewidth=2))
    fill_between(noHogTrainedStats[0],
                 noHogTrainedStats[1] - noHogTrainedStats[3],
                 noHogTrainedStats[1] + noHogTrainedStats[3],
                 color='darkorchid',
                 alpha=0.5)
    labels.append('SCATAT no HOG')
    

  # Plot the ignore time training
  if ignoreTimeStats is not None:
    elip = Ellipse((ignoreTimeStats[0], ignoreTimeStats[3]),
                             height=ignoreTimeStats[5]*2,
                             width=ignoreTimeStats[2]*2,
                             alpha=0.5,
                             color='darkred')
    gca().add_artist(elip)
    labels.append('SCATAT Ignore Time')
    scatLines.append(elip)

  gca().autoscale_view(scaley=True)
  xlabel(xLabel)
  ylabel(yLabel)
  legend(scatLines, labels, loc=legendLoc)
  return fig

def FScore(precision, recall):
  if (precision + recall) < 1e-7:
    return 0
  return 2*precision*recall/(precision+recall)

def CalculateAveragePrecision(precision, recall, vuNames, hogPR):
  # Get the starting point of the curve
  maxRecall = np.nanmax(recall)
  rospy.loginfo("Maximum Average Precision Possible: %f" % maxRecall)

  hogRecall = hogPR[1]
  hogPrecision = hogPR[0]
  uniqueRecall = np.unique(hogRecall)
  validIdx = np.searchsorted(hogRecall, uniqueRecall)
  validIdx.sort()
  hogRecall = np.concatenate(([0],hogRecall[validIdx]))
  hogPrecision = hogPrecision[validIdx]
  hogPrecision = np.concatenate(([hogPrecision[-1]], hogPrecision))
  hogInterp = scipy.interpolate.interp1d(hogRecall, hogPrecision, 'cubic')

  # Calculate for the HOG baseline result
  hogRecall = hogPR[1]
  hogPrecision = hogPR[0][hogRecall <= maxRecall]
  hogRecall = hogRecall[hogRecall <= maxRecall]
  rospy.loginfo("HOG: %f" % scipy.integrate.trapz(hogPrecision, hogRecall))

  for i in range(len(vuNames)):
    curPrecision = precision[0][i]
    curRecall = recall[0][i][np.isfinite(curPrecision)]
    curPrecision = curPrecision[np.isfinite(curPrecision)]
    sortIdx = np.argsort(curRecall)
    curRecall = curRecall[sortIdx]
    curPrecision = curPrecision[sortIdx]
    curHogPrecision = hogInterp(curRecall)
    hogAvgPrecision = scipy.integrate.trapz(curHogPrecision, curRecall)
    vuAvgPrecision = scipy.integrate.trapz(curPrecision, curRecall)
    rospy.loginfo("%s: %f" % (vuNames[i],
                              vuAvgPrecision / hogAvgPrecision))

def CalculateAveragePrecision01(precision, recall, vuNames):
  rospy.loginfo("Calculating the average precision of the VU Estimator")
  for i in range(len(vuNames)):
    curPrecision = precision[0][i]
    curRecall = recall[0][i][np.isfinite(curPrecision)]
    curPrecision = curPrecision[np.isfinite(curPrecision)]
    sortIdx = np.argsort(curRecall)
    curPrecision = curPrecision[sortIdx]
    curRecall = curRecall[sortIdx]
    curRecall = np.concatenate(([0], curRecall, [1]))
    curPrecision = np.concatenate(([curPrecision[0]], curPrecision,
                                   [curPrecision[-1]]))

    avgPrecision = scipy.integrate.trapz(curPrecision, curRecall)
    rospy.loginfo('%s: %f' % (vuNames[i], avgPrecision))

def GetVarianceOfOnePerson(peopleCount, fracLeft):
  eCount = peopleCount / fracLeft
  if eCount < 1:
    return eCount - eCount*eCount

  return 0

def CalculateResamplingModel(options, windowCount, baselineTime=None):
  nWindows, maxHogTime, EstimateTime = PlottingUtils.ParseHogTiming(options)
  if baselineTime is None:
    baselineTime = maxHogTime
  nPeople = len(windowCount)
  fracWindows = np.exp(np.arange(0.0, 6, 0.01))
  nonZeroCounts = filter(lambda x: x>0, windowCount)
  nFound = np.array([sum([min(count/frac,1) for count in nonZeroCounts])
            for frac in fracWindows])
  #nFound = np.append(nFound, [0.0])
  meanRecall = np.divide(nFound, nPeople)
  nVariance = np.array([sum([GetVarianceOfOnePerson(count, frac)
                             for count in nonZeroCounts])
                        for frac in fracWindows])
  #nVariance = np.append(nVariance, [0.0])
  stdRecall = np.divide(np.sqrt(nVariance), nPeople)
  windowCount = np.array([nWindows/frac for frac in fracWindows])
  #windowCount = np.append(windowCount, [0.0])
  processingTime = EstimateTime(windowCount)
  speedup = np.divide(baselineTime, processingTime)

  return (speedup, meanRecall, stdRecall, windowCount, processingTime)

def CalculateNumberOfDetectorsForSameSpeed(resamplingModel, vuRecall,
                                           vuNWindows, vuProcessingTime):
  sortIdx = np.argsort(resamplingModel[1])
  resampleNWindows = scipy.interpolate.interp1d(resamplingModel[1][sortIdx],
                                                resamplingModel[3][sortIdx],
                                                'cubic')
  resampleTime = scipy.interpolate.interp1d(resamplingModel[1][sortIdx],
                                            resamplingModel[4][sortIdx],
                                            'cubic')

  # We calculated this value experimentally. By timing the HOG
  # detector and profiling the time spent in the hog model evaluation.
  HOG_TIME_PER_WINDOW = 4.8e-6

  vuRecallCopy = vuRecall.copy()
  vuRecallCopy[vuRecallCopy < np.min(resamplingModel[1])] = float('nan')

  deltaT = vuProcessingTime - resampleTime(vuRecallCopy)
  deltaT[deltaT < 0] = float('nan')

  deltaW = resampleNWindows(vuRecallCopy) - vuNWindows
  deltaW[deltaW < 0] = float('nan')

  nDetectors = deltaT / (HOG_TIME_PER_WINDOW * deltaW)

  # Hack so that there's an entry at 0 recall so that the scaling
  # doesn't get messed up
  nDetectors[:,:,-1] = 1000

  return nDetectors + 1

def AddLineToPlot(fig, x, y, marker='', color='b', label='', legendLoc=1):
  legendLabels = [text.get_text() for text in
                  fig.axes[0].get_legend().get_texts()]
  legendLines = [line for line in fig.axes[0].get_lines()]
  legendLines = legendLines[0:len(legendLabels)]
  #legendLines, legendLabels = fig.axes[0].get_legend_handles_labels()
  legendLines.append(plot(x,
                          y, hold=True,
                          marker=marker, color=color))
  legendLabels.append(label)
  legend(legendLines, legendLabels, loc=legendLoc)
    

if __name__ == '__main__':
  parser = OptionParser(usage=usage)
  parser.add_option('--input_data', '-i', default=None,
                    help='File containing the pickled data so that it does not need to be calculated again. Can be comma separated for multiple files. If that is the case, the first one must be the slower version.')
  parser.add_option('--output_prefix', default=None,
                    help='If the graphs should be saved to file, what is the prefix')
  parser.add_option('--hog_timing_file',
                    default='/data/mdesnoye/pedestrian/vu_estimation/eth/hog_cached/hog_cached_timing.txt',
                    help='File that specifies the timing for the hog.')
  parser.add_option('--group_names',
                    default=None,
                    help='Comma separated names for the legend in the graph')
  parser.add_option('--valid_entries', default=None,
                    help='Python expression specifying what datasets to plot')
  parser.add_option('--trained_stats', default=None,
                    help='Glob specifying stats files for a trained approach.')
  parser.add_option('--trained_regex',
                    default='IntegralHOGCascade.*_([0-9A-Za-z]+)_[0-9A-Za-z\.]+\.stats',
                    help='Regex to extract the data of the trained stats so that we can merge the statistics.')
  parser.add_option('--no_hog_trained_stats', default=None,
                    help='Glob specifying stats files for a trained approach with no HOG detector.')
  parser.add_option('--ignore_time_stats', default=None,
                    help='Glob specifying stats files for the ignore time when training scenario.')
  parser.add_option('--ignore_time_regex',
                    default='IntegralHOGCascade.*_([0-9A-Za-z]+)\.stats',
                    help='Regex to extract the data of the ignore time stats so that we can merge the statistics.')

  (options, args) = parser.parse_args()

  # Parse the input data
  inputFiles = options.input_data.split(',')
  stats = None
  slowHogProcessingTime = float('inf')
  precision = []
  for inputFile in inputFiles:
    if inputFile is not None and os.path.exists(inputFile):
      rospy.loginfo('Loading statistics from %s' % inputFile)
      curStats = pickle.load(open(inputFile, 'rb'))
      rospy.loginfo('Statistcs loaded. Object is version %s' %
                    curStats.VERSION)

      if stats is None:
        stats = curStats
        slowHogProcessingTime = stats.hogProcessingTime
        stats.processingTime = np.expand_dims(stats.processingTime, 0)
        stats.precision = np.expand_dims(stats.precision, 0)
        stats.recall = np.expand_dims(stats.recall, 0)
        stats.accuracy = np.expand_dims(stats.accuracy, 0)
        stats.errorRate = np.expand_dims(stats.errorRate, 0)
        stats.nWindows = np.expand_dims(stats.nWindows, 0)
        stats.matthewsCoefficients = np.expand_dims(
          stats.matthewsCoefficients, 0)
        stats.vuNegPrecision = np.expand_dims(stats.vuNegPrecision, 0)
        stats.vuPrecision = np.expand_dims(stats.vuPrecision, 0)
        stats.vuRecall = np.expand_dims(stats.vuRecall, 0)
        stats.hogProcessingTime = np.tile(stats.hogProcessingTime,
                                          stats.precision.shape)
      else:
        # Merge the stats
        stats.hogProcessingTime = np.concatenate(
          (stats.hogProcessingTime, np.tile(curStats.hogProcessingTime,
                                            stats.precision.shape)))
        stats.processingTime = np.concatenate(
          (stats.processingTime, np.expand_dims(curStats.processingTime, 0)),
           0)
        stats.precision = np.concatenate(
          (stats.precision, np.expand_dims(curStats.precision, 0)), 0)
        stats.recall = np.concatenate(
          (stats.recall, np.expand_dims(curStats.recall, 0)), 0)
        stats.accuracy = np.concatenate(
          (stats.accuracy, np.expand_dims(curStats.accuracy, 0)), 0)
        stats.errorRate = np.concatenate(
          (stats.errorRate, np.expand_dims(curStats.errorRate, 0)), 0)
        stats.nWindows = np.concatenate(
          (stats.nWindows, np.expand_dims(curStats.nWindows, 0)), 0)
        stats.matthewsCoefficients = np.concatenate(
          (stats.matthewsCoefficients,
           np.expand_dims(curStats.matthewsCoefficients, 0)), 0)
        stats.vuNegPrecision = np.concatenate(
          (stats.vuNegPrecision, np.expand_dims(curStats.vuNegPrecision, 0)),
           0)
        stats.vuPrecision = np.concatenate(
          (stats.vuPrecision, np.expand_dims(curStats.vuPrecision, 0)), 0)
        stats.vuRecall = np.concatenate(
          (stats.vuRecall, np.expand_dims(curStats.vuRecall, 0)), 0)

  # Parse the stats for a trained approach
  trainedStats = TrainedStats(options.trained_regex, slowHogProcessingTime)
  if options.trained_stats is not None:
    for statFile in glob.iglob(options.trained_stats):
      trainedStats.AddStats(statFile)

  # Parse the stats for the trained approach that doesn't have a HOG detector
  noHogTrainedStats = TrainedStats(options.trained_regex, slowHogProcessingTime)
  if options.no_hog_trained_stats is not None:
    for statFile in glob.iglob(options.no_hog_trained_stats):
      noHogTrainedStats.AddStats(statFile)

  # Parse the stats for the ignoring the time
  ignoreStats = TrainedStats(options.ignore_time_regex, slowHogProcessingTime)
  if options.ignore_time_stats is not None:
    for statFile in glob.iglob(options.ignore_time_stats):
      ignoreStats.AddStats(statFile)

  # Figure out which datasets to plot
  if options.valid_entries is None:
    validEntries = None
  else:
    validEntries = eval(options.valid_entries)

  # Calculate the model for the resampling strategy (speedup,
  # meanRecall, stdRecall)
  resamplingModel = CalculateResamplingModel(options,
                                             stats.peopleWindowCount,
                                             slowHogProcessingTime)

  groupColors = ['b', 'g', 'r', 'k', 'm', 'y', '0.5', 'darkturquoise']
  groupMarkers = ['o', '+', 'x', '^', 's', 'p', 'D', '<']

  if options.group_names is not None:
    groupNames = options.group_names.split(',')
  else:
    groupNames = [x for x in stats.vuTypes]
  if stats.nVUEstimators > len(groupNames):
    groupNames.append('Resample Boxes')
    
  # Plot the precision vs. speedup
  PlotAccuracyScatter([slowHogProcessingTime / x[6] for
                       x in stats.scaledHogStats],
                      [x[1] for x in stats.scaledHogStats],
                      slowHogProcessingTime / stats.processingTime,
                      stats.precision,
                      groupColors,
                      groupMarkers,
                      groupNames,
                      'Speed Up',
                      'Precision',
                      stats.nVUEstimators,
                      logX=True,
                      legendLoc=4,
                      validEntries=validEntries,
                      trainedStats=trainedStats.GetSpeedupVsMetric(
                        lambda x: x.precision[0]),
                      noHogTrainedStats=noHogTrainedStats.GetVuSpeedupVsMetric(
                        lambda x: x.vuPrecision[0]),
                      ignoreTimeStats=ignoreStats.GetVuSpeedupVsPointMetric(
                        lambda x: x.vuPrecision[0]))
  if options.output_prefix is not None:
    savefig(options.output_prefix + '_Precision.eps')
    savefig(options.output_prefix + '_Precision.png')

  # recall vs. speedup
  recallFig = PlotAccuracyScatter([slowHogProcessingTime / x[6] for
                                   x in stats.scaledHogStats],
                                  [x[2] for x in stats.scaledHogStats],
                                  slowHogProcessingTime / stats.processingTime,
                                  stats.recall,
                                  groupColors,
                                  groupMarkers,
                                  groupNames,
                                  'Speed Up',
                                  'Recall',
                                  stats.nVUEstimators,
                                  logX=True,
                                  validEntries=validEntries,
                                  trainedStats=trainedStats.GetSpeedupVsMetric(
                                    lambda x: x.recall[0]),
                                  noHogTrainedStats=noHogTrainedStats.GetVuSpeedupVsMetric(
                                    lambda x: x.vuRecall[0]),
                                  ignoreTimeStats=ignoreStats.GetVuSpeedupVsPointMetric(
                                    lambda x: x.vuRecall[0]))
  AddLineToPlot(recallFig, resamplingModel[0], resamplingModel[1],
                '.', 'chocolate', 'Resampling Initial Boxes')
  fill_between(resamplingModel[0], resamplingModel[1] - resamplingModel[2],
               resamplingModel[1] + resamplingModel[2], color='chocolate',
               alpha=0.5)
  if options.output_prefix is not None:
    savefig(options.output_prefix + '_Recall.eps')
    savefig(options.output_prefix + '_Recall.png')

  # Error rate vs. accuracy
  #PlotAccuracyScatter([x[0] for x in groundTruths],
  #                    [x[3] for x in groundTruths],
  #                    hogProcessingTimes[0] / processingTime,
  #                    #accuracy,
  #                    errorRate,
  #                    groupColors,
  #                    groupMarkers,
  #                    groupNames,
  #                    'Speed Up',
  #                    'Error Rate',
  #                    nVUEstimators)

  # F-score vs. accuracy
  fScoreFunc = lambda x: (2 * x.precision[0] * x.recall[0] /
                          (x.precision[0] + x.recall[0]))
  vuFScoreFunc = lambda x: (2 * x.vuPrecision[0] * x.vuRecall[0] /
                            (x.vuPrecision[0] + x.vuRecall[0]))
  fFig = PlotAccuracyScatter([slowHogProcessingTime / x[6] for
                              x in stats.scaledHogStats],
                             [FScore(x[1], x[2]) for x in stats.scaledHogStats],
                             slowHogProcessingTime / stats.processingTime,
                             2*stats.precision*stats.recall /
                             (stats.precision+stats.recall),
                             groupColors,
                             groupMarkers,
                             groupNames,
                             'Speed Up',
                             'F-Score',
                             stats.nVUEstimators,
                             logX=True,
                             validEntries=validEntries,
                             trainedStats=trainedStats.GetSpeedupVsMetric(
                               fScoreFunc),
                             noHogTrainedStats=noHogTrainedStats.GetVuSpeedupVsMetric(
                               vuFScoreFunc),
                             ignoreTimeStats=ignoreStats.GetVuSpeedupVsPointMetric(
                               vuFScoreFunc)
                             )
  maxPrecision = max(stats.precision[0, :, 0])
  AddLineToPlot(fFig, resamplingModel[0],
                2*resamplingModel[1]*maxPrecision /
                (resamplingModel[1] + maxPrecision),
                '.', 'chocolate', 'Resampling Initial Boxes',
                legendLoc=3)
  lowerRecall = resamplingModel[1] - resamplingModel[2]
  lowerFscore = 2 * lowerRecall * maxPrecision / (maxPrecision + lowerRecall)
  upperRecall = resamplingModel[1] + resamplingModel[2]
  upperFscore = 2 * upperRecall * maxPrecision / (maxPrecision + upperRecall)
  fill_between(resamplingModel[0], lowerFscore, upperFscore,
               color='chocolate', alpha=0.5)
  if options.output_prefix is not None:
    savefig(options.output_prefix + '_fscore.eps')
    savefig(options.output_prefix + '_fscore.png')

  # Number of windows evaluated
  #PlotAccuracyScatter([slowHogProcessingTime / x[6] for
  #                     x in stats.scaledHogStats],
  #                    [x[4] for x in stats.scaledHogStats],
  #                    slowHogProcessingTime / stats.processingTime,
  #                    stats.nWindows,
  #                    groupColors,
  #                    groupMarkers,
  #                    groupNames,
  #                    'Speed Up',
  #                    'N Windows',
  #                    stats.nVUEstimators)

  # Negative precision of the visual utility vs. speedup
  #PlotAccuracyScatter([slowHogProcessingTime / x[6] for
  #                     x in stats.scaledHogStats],
  #                     [x[7] for x in stats.scaledHogStats],
  #                     slowHogProcessingTime / stats.processingTime,
  #                     stats.vuNegPrecision,
  #                     groupColors,
  #                     groupMarkers,
  #                     groupNames,
  #                     'Speedup',
  #                     'Negative Precision',
  #                     stats.nVUEstimators,
  #                     validEntries=validEntries)

  # Timing
  PlotAccuracyScatter([x[4] for x in stats.scaledHogStats],
                      [x[6] for x in stats.scaledHogStats],
                      stats.nWindows,
                      stats.processingTime,
                      groupColors,
                      groupMarkers,
                      groupNames,
                      '# of Windows',
                      'Processing Time (s)',
                      stats.nVUEstimators,
                      validEntries=validEntries,
                      trainedStats=trainedStats.GetMetric(
                        lambda x: x.nWindows[0],
                        lambda x: x.meanProcessingTime[0]))

  # Matthews coefficient
  #PlotAccuracyScatter([slowHogProcessingTime / x[6] for
  #                     x in stats.scaledHogStats],
  #                    [x[5] for x in stats.scaledHogStats],
  #                    slowHogProcessingTime / stats.processingTime,
  #                    stats.matthewsCoefficients,
  #                    groupColors,
  #                    groupMarkers,
  #                    groupNames,
  #                    'Speed Up',
  #                    'Matthrews Coefficients',
  #                    stats.nVUEstimators)
  #if options.output_prefix is not None:
  #  savefig(options.output_prefix + '_matt.eps')
  #  savefig(options.output_prefix + '_matt.png')

  # Precision/Recall Curve
  prFig = PlotAccuracyScatter([x[2] for x in stats.scaledHogStats],
                              [x[1] for x in stats.scaledHogStats],
                              stats.recall[0:1],
                              stats.precision[0:1],
                              groupColors,
                              groupMarkers,
                              groupNames,
                              'Recall',
                              'Precision',
                              stats.nVUEstimators,
                              validEntries=validEntries,
                              trainedStats=trainedStats.GetMetric(
                                lambda x: x.recall[0],
                                lambda x: x.precision[0]),
                              noHogTrainedStats=noHogTrainedStats.GetMetric(
                                lambda x: x.vuRecall[0],
                                lambda x: x.vuPrecision[0]),
                              ignoreTimeStats=ignoreStats.GetPointMetric(
                                lambda x: x.vuRecall[0],
                                lambda x: x.vuPrecision[0]))
  AddLineToPlot(prFig, stats.hogPRCurves[0][1], stats.hogPRCurves[0][0],
                '.', 'chocolate', 'Adjusting HOG Threshold', 8)
  if options.output_prefix is not None:
    savefig(options.output_prefix + '_PR.eps')
    savefig(options.output_prefix + '_PR.png')

  
  CalculateAveragePrecision(stats.precision, stats.recall, groupNames,
                            stats.hogPRCurves[0])

  # Precision/Recall Curve of the VU Estimators
  vuPrFig = PlotAccuracyScatter([x[8] for x in stats.scaledHogStats],
                                [x[1] for x in stats.scaledHogStats],
                                stats.vuRecall[0:1],
                                stats.vuPrecision[0:1],
                                groupColors,
                                groupMarkers,
                                groupNames,
                                'Recall',
                                'Precision',
                                stats.nVUEstimators,
                                logY=True,
                                validEntries=validEntries)
  maxVuPrecision = max(stats.vuPrecision[0, :, 0])
  AddLineToPlot(vuPrFig, [0.0, 1.0], [maxVuPrecision, maxVuPrecision],
                '.', 'chocolate', 'Resampling Initial Boxes')
  CalculateAveragePrecision01(stats.vuPrecision, stats.vuRecall, groupNames)
  if options.output_prefix is not None:
    savefig(options.output_prefix + '_vuPr.eps')
    savefig(options.output_prefix + '_vuPr.png')

  # The number of HOG detectors that need to be applied to get the
  # same speedup as the resampling strategy.
  nDetectors = CalculateNumberOfDetectorsForSameSpeed(
    resamplingModel,
    stats.recall,
    stats.nWindows,
    stats.processingTime)
  PlotAccuracyScatter([0 for x in stats.scaledHogStats],
                      [0 for x in stats.scaledHogStats],
                      stats.recall,
                      nDetectors,
                      groupColors,
                      groupMarkers,
                      groupNames,
                      'Recall',
                      'Number of HOG Detectors to be as Fast as a Resampling Strategy',
                      stats.nVUEstimators,
                      logY = True,
                      validEntries=validEntries)
  if options.output_prefix is not None:
    savefig(options.output_prefix + '_break.eps')
    savefig(options.output_prefix + '_break.png')

  # Number of windows for a given recall level
  curFig = PlotAccuracyScatter([0 for x in stats.scaledHogStats],
                               [0 for x in stats.scaledHogStats],
                               stats.recall[0:1],
                               stats.nWindows[0:1],
                               groupColors,
                               groupMarkers,
                               groupNames,
                               'Recall',
                               'Number of Windows',
                               stats.nVUEstimators,
                               validEntries=validEntries)
  AddLineToPlot(curFig, resamplingModel[1], resamplingModel[3],
                '.', 'chocolate', 'Resampling Initial Boxes', 2)
  if options.output_prefix is not None:
    savefig(options.output_prefix + '_winVrecall.eps')
    savefig(options.output_prefix + '_winVrecall.png')

  # Calculate the timing of just using the visual utility estimator
  garb1, garb2, EstimateTime = PlottingUtils.ParseHogTiming(options)
  vuProcessingTime = stats.processingTime - EstimateTime(stats.nWindows)

  # Bayes risk vs. speedup
  PlotAccuracyScatter([slowHogProcessingTime / x[6] for
                       x in stats.scaledHogStats],
                      [x[1] for x in stats.scaledHogStats],
                      (stats.accuracy / np.max(stats.accuracy)) +
                      (stats.processingTime / slowHogProcessingTime),
                      stats.recall,
                      groupColors,
                      groupMarkers,
                      groupNames,
                      'Bayes Risk * Processing Time',
                      'Recall',
                      stats.nVUEstimators,
                      doSort=False,
                      validEntries=validEntries)

  curFig = PlotAccuracyScatter([slowHogProcessingTime / x[6] for
                                x in stats.scaledHogStats],
                               [x[1] for x in stats.scaledHogStats],
                               slowHogProcessingTime / stats.processingTime,
                               (stats.accuracy / np.max(stats.accuracy))+
                               (stats.processingTime / slowHogProcessingTime),
                               groupColors,
                               groupMarkers,
                               groupNames,
                               'Speedup',
                               'Visual Utility Risk',
                               stats.nVUEstimators,
                               validEntries=validEntries)
  AddLineToPlot(curFig, resamplingModel[0],
                resamplingModel[4] / slowHogProcessingTime +
                (1-resamplingModel[3] / resamplingModel[3][0]),
                '.', 'chocolate', 'Resampling Initial Boxes')
  if options.output_prefix is not None:
    savefig(options.output_prefix + '_VuRisk.eps')
    savefig(options.output_prefix + '_VuRisk.png')
  
  
  show()
  
