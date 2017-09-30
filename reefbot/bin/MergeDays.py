#!/usr/bin/python
import roslib; roslib.load_manifest('reefbot')
import rospy
import rosbag
import csv
import math
import re
from optparse import OptionParser

if __name__ == '__main__':
  # All of the command line flags
  parser = OptionParser()

  (options, args) = parser.parse_args()

  inFile = args[0]
  outFile = args[1]

  dateRegex = re.compile(r'([0-9]{4}-[0-9]{2}-[0-9]{2})')

  inStream = csv.reader(open(inFile, 'rb'))
  outStream = csv.writer(open(outFile, 'wb'))
  curRow = None
  for row in inStream:
    if curRow is None:
      curRow = row
      continue

    rowDate = dateRegex.search(row[0])
    curDate = dateRegex.search(curRow[0])
    if (curDate is None or rowDate is None or
        curDate.group(1) <> rowDate.group(1)):
      # A new date so print out the last one
      outStream.writerow(curRow)
      curRow = row
      curRow[0] = rowDate.group(1)
    else:
      # The date is the same so merge them
      for i in range(1,len(row)):
        curRow[i] = float(curRow[i]) + float(row[i])
      
      
      
