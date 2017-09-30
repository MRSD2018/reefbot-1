#!/usr/bin/python
# The species ids changed on me, so this script converts old bag files
# to contain the new species ids
#
# Author: Mark Desnoyer (markd@cmu.edu)

import roslib; roslib.load_manifest('reefbot_msgs')
import rosbag
from optparse import OptionParser
from reefbot_msgs.msg import UserSpeciesSelection
from reefbot_msgs.msg import SpeciesScore

# Lookup table from the old id to the new one. New ids are from
# revision 280 in the repo
idLUT = {
  1 : 16,
  2 : 14,
  3 : 8,
  4 : 22,
  5 : 9,
  6 : 31,
  7 : 32,
  8 : 20,
  9 : 29,
  10: 51,
  11: 28,
  12: 11,
  13: 10,
  14: 11,
  15: 18,
  16: 35,
  17: 25,
  18: 36,
  19: 21,
  20: 5,
  21: 38,
  22: 4,
  23: 7,
  24: 6,
  25: 34,
  26: 39,
  27: 13,
  28: 40,
  29: 17,
  30: 41,
  31: 19,
  32: 12,
  33: 37,
  34: 42,
  35: 43,
  36: 24,
  37: 44,
  38: 24,
  39: 45,
  40: 46,
  41: 47,
  42: 48,
  43: 49,
  44: 50}

def ConvertSpeciesId(oldId):
  return idLUT[oldId]

if __name__=='__main__':
  parser = OptionParser()

  parser.add_option('--input', '-i', dest='input',
                    help='filename of the input bag',
                    default=None)

  parser.add_option('--output', '-o', dest='output',
                    help='filename of the output bag',
                    default=None)

  (options, args) = parser.parse_args()

  with rosbag.Bag(options.output, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(options.input).read_messages():
      # For the UserSpeciesSelection message type
      try:
        msg.species_id = ConvertSpeciesId(msg.species_id)
      except AttributeError: pass

      # For the SpeciesScore message type
      try:
        for answer in msg.answers:
          for species in answer.best_species:
            species.species_id = ConvertSpeciesId(species.species_id)
      except AttributeError: pass

      outbag.write(topic, msg, t)
