#!/usr/bin/python
'''Plots the processing time vs. number of windows from a timing file.'''
usage='PlotComputationModel.py <timing file>'

import sys
from pylab import *
import numpy as np

if __name__ == '__main__':
  nWindows = []
  times = []
  for line in open(sys.argv[1]):
    splitLine = line.strip().split(',')
    nWindows.append(int(splitLine[0]))
    times.append(float(splitLine[1]))
  times = np.array(times)
  nWindows = np.array(nWindows)

  figure()
  plot(nWindows, np.max(times) - times, linewidth=2.0)
  xlabel('# of Windows Processed')
  ylabel('Processing Time (s)')

  show()
