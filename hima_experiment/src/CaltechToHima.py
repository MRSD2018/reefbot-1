#! /usr/bin/python
'''A script that converts a labeled video sequence in VBB/SEQ format to HIMA.'''
usage='CaltechToHima.py [options] <vbb_dir> <seq_dir> <outputDir>'

from optparse import OptionParser
import os
import os.path
import subprocess
import re

if __name__ == '__main__':
  parser = OptionParser(usage=usage)

  (options, args) = parser.parse_args()

  vbbDir = args[0]
  seqDir = args[1]
  outputDir = args[2]

  # Create the output directories
  if not os.path.exists(outputDir):
    os.makedirs(outputDir)

  # Get the list of vbb/seq files in the directory
  sequences = []
  seqRegex = re.compile('([0-9a-zA-Z]*)\.seq')
  for filename in os.listdir(seqDir):
    match = seqRegex.match(filename)
    if match:
      base = match.groups()[0]
      vbbFile = os.path.join(vbbDir, base + '.vbb')
      if not os.path.exists(vbbFile):
        raise Exception('Could not find corresponding vbb file: %s'
                        % (vbbFile))
      curOutputDir = os.path.join(outputDir, base)
      sequences.append((curOutputDir, vbbFile, os.path.join(seqDir, filename)))


  for curOutput, vbbFile, seqFile in sequences:
    # Build the output directores
    if not os.path.exists(curOutput):
      os.makedirs(curOutput)
    frameDir = os.path.join(curOutput, 'frames')
    if not os.path.exists(frameDir):
      os.makedirs(frameDir)

    # Run the matlab command that will do the conversion
    matlabCmd = ("addpath(genpath('src'));,"
                 "vbbToHima('%s', '%s/annotations.txt');,"
                 "seqIo('%s', 'toImgs', '%s');"
                 % (vbbFile, curOutput, seqFile, frameDir))
               
    cmd = ["matlab", "-nosplash", "-nodesktop", "-r",
           "try, %s, catch ME, ME.getReport(), end, quit" % matlabCmd]
  
    retcode = subprocess.call(cmd)

    if retcode <> 0:
      raise Exception("Error running the command: %s" % ' '.join(cmd))

  
