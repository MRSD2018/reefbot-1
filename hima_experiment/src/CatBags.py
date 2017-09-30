#!/usr/bin/python
import roslib; roslib.load_manifest('hima_experiment')
import rospy
import rosbag
from optparse import OptionParser

usage='CatBags.py -o <outputBag> <inputBag0> <inputBag1> ...'

if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  parser.add_option('-o', '--output', dest='output',
                    help='Comma separated list of directories containing the results of the experiment',
                    default='.')

  (options, args) = parser.parse_args()

  # Open up the output bag
  with rosbag.Bag(options.output, 'w') as outBag:
    for inFile in args:
      print 'Adding in %s' % inFile
      with rosbag.Bag(inFile) as inBag:
        for topic, msg, t in inBag.read_messages():
          outBag.write(topic, msg, t)
