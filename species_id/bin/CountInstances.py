#!/usr/bin/python

import sys
from optparse import OptionParser
import csv

if __name__ == '__main__':
  parser = OptionParser()

  parser.add_option('--input', '-i',
                    help='Input file',
                    default=None)

  parser.add_option('--output', '-o',
                    help='Output file',
                    default=None)

  (options, args) = parser.parse_args()

  sums = []
  for i in range(60):
    sums.append(0)

  for line in open(options.input):
    name, fishId = line.strip().split(',')
    fishId = int(fishId)
    sums[fishId] = sums[fishId] + 1

  outputFile = open(options.output, 'w')
  outputFile.write('id,count\n')
  i = 0
  for s in sums:
    if s > 0:
      outputFile.write('%i,%i\n' % (i, s))
    i = i + 1
