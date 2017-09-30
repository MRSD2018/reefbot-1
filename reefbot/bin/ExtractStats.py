#!/usr/bin/python
'''Program that parses a bunch of bag files and collects stats on a daily basis.
'''
usage='ExtractStats.py [options] <bag0> <bag1> ...'

import roslib; roslib.load_manifest('reefbot')
import rospy
import rosbag
import csv
import math
import re
from optparse import OptionParser

class StatCollector:
  '''Abstract class for collecting stats'''
  def __init__(self):
    self.count = 0

  def ResetCount(self):
    self.count = 0

  def _IncrementCount(self, amount=1):
    self.count += amount;

  def GetCount(self):
    return self.count

  def ProcessMessage(self, topic, message, time):
    '''Process a ROS message and call _IncrementCount as necessary.'''
    raise NotImplementedError()

class Runtime(StatCollector):
  '''Keeps track of the number of minutes of uptime.'''
  def __init__(self):
    StatCollector.__init__(self)
    self.lastMinute = 0;

  def ProcessMessage(self, topic, message, time):
    try:
      curSeconds = message.header.stamp.secs
      curMinute = math.floor(curSeconds / 60)
      diff = curMinute - self.lastMinute
      if diff < 3 and diff > 0:
        self._IncrementCount(diff)
      elif diff < 0:
        rospy.logwarn("We got a message from the past")
      self.lastMinute = curMinute
    except AttributeError as e:
      pass

class UserActiveTime(StatCollector):
  '''Keeps track of the number of minutes of user time.'''
  def __init__(self):
    StatCollector.__init__(self)
    self.lastMinute = 0;

  def ProcessMessage(self, topic, message, time):
    if topic == '/universal_joy':
      try:
        curSeconds = time.to_sec()
        curMinute = math.floor(curSeconds / 60)
        diff = curMinute - self.lastMinute
        if diff < 3 and diff > 0:
          self._IncrementCount(diff)
        elif diff < 0:
          rospy.logwarn("We got a message from the past")
        self.lastMinute = curMinute
      except AttributeError as e:
        pass

class PowerOnCount(StatCollector):
  '''Counts the number of times the robot was turned on.'''
  def __init__(self):
    StatCollector.__init__(self)

  def ProcessMessage(self, topic, message, time):
    if topic == '/rosout':
      if (message.name == '/CameraCaptureManager' and
          message.msg == 'Initialized Camera Capture Manager'):
        self._IncrementCount(1)

class ImagesTaken(StatCollector):
  def __init__(self):
    StatCollector.__init__(self)

  def ProcessMessage(self, topic, message, time):
    if topic == '/still_image':
      self._IncrementCount(1)

class HumanPicturesFound(StatCollector):
  def __init__(self):
    StatCollector.__init__(self)
    
  def ProcessMessage(self, topic, message, time):
    if topic == '/species_id':
      if (len(message.answers) > 0 and
          len(message.answers[0].best_species) > 0):
        if (message.answers[0].best_species[0].species_id == 47):
          self._IncrementCount(1)

class KeyPresses(StatCollector):
  def __init__(self):
    StatCollector.__init__(self)
    
  def ProcessMessage(self, topic, message, time):
    if topic == '/universal_joy':
      self._IncrementCount(1)

      

def RecordRow(date, collectors):
  curRow = [date]
  for collector in collectors:
    curRow.append(collector.GetCount())
    collector.ResetCount()

  return curRow

if __name__ == '__main__':
  # All of the command line flags
  parser = OptionParser(usage=usage)

  parser.add_option('--collectors',
                    help='Python list of collector objects to use',
                    default='[Runtime(),UserActiveTime(),ImagesTaken(),HumanPicturesFound(),KeyPresses(),PowerOnCount()]')
  parser.add_option('--output', '-o',
                    help='Output CSV File',
                    default='stats.out')

  (options, args) = parser.parse_args()

  exec("collectors = %s" % options.collectors)

  # Open the output file
  outputFile = csv.writer(open(options.output, 'wb'))
  outputFile.writerow(['Date'] + [x.__class__.__name__ for x in collectors])

  # Go through each of the bags and start collecting
  lastDate = None
  dateRegex = re.compile('message_log_([0-9]{4}(-[0-9]{2}){2}).*\.bag')
  for bagfile in args:
    # Open the bag
    bag = rosbag.Bag(bagfile, 'r')
    try:
      print "Processing %s" % bagfile
    
      date = dateRegex.search(bagfile).group(1)
      if date <> lastDate:
        if lastDate is not None:
          # New date in the bags, so we need to store the current values
          outputFile.writerow(RecordRow(lastDate, collectors))
      
        lastDate = date

      # Now pass each message from the bag into the collectors
      for topic, msg, t in bag.read_messages():
        for collector in collectors:
          collector.ProcessMessage(topic, msg, t)
    except Exception:
      pass
    finally:
      bag.close()

  outputFile.writerow(RecordRow(lastDate, collectors))
