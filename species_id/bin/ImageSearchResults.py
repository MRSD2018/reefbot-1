import csv
import numpy as np
import os.path
import os
import re

def GetMixingFiles(experimentDir, videoRegexp, shapeWeightRegexp, fileRegexp):
  '''Looks in the experiment directory to find all the files with results from the mixing experiment.

  Returns: A list of tuples (filename, shape weighting, video id)
  '''
  retVal = []
  for filename in os.listdir(experimentDir):
    if fileRegexp.match(filename):
      shapeWeight = float(shapeWeightRegexp.search(filename).groups()[0])
      videoId = videoRegexp.search(filename).groups()[0]
      retVal.append((filename, shapeWeight, videoId))

  return retVal

def ParseMixingResults(experimentDir, mixingFiles):
  '''Opens up all the mixing results and puts them in a nice data structure.

  Returns: A map from (videoId, mixing%) -> ([(filenames, blobId)], [humanLabels], [([machineLabels], [machineScores], [closestImage])])
  '''
  retval = {}
  for mixingFile in mixingFiles:
    retval[(mixingFile[2], mixingFile[1])] = ParseResultsFile(
      os.path.join(experimentDir, mixingFile[0]))

  return retval

def ParseResultsFile(resultFile):
  '''Parses a results file.

  Returns: A tuple of ([(filenames, blobId)], [humanLabels], [([machineLabels], [machineScores], [closestImage])])
  '''
  f = open(resultFile)
  try:
    reader = csv.reader(f)
      
    blobs = []
    humanLabels = []
    machineResults = None

    # Skip the first line (column labels)
    reader.next()

    # Parse each line
    for line in reader:
      blobs.append((line[0], int(line[1])))
      humanLabels.append(int(line[2]))
      nLabels = (len(line) - 3) / 3 
      if machineResults is None:
        machineResults = [([], [], []) for x in range(nLabels)]
      for i in range(nLabels):
        machineResults[i][0].append(int(line[3 + 3*i]))
        machineResults[i][1].append(float(line[4 + 3*i]))
        machineResults[i][2].append(line[5 + 3*i])

    for i in range(nLabels):
      machineResults[i] = (np.array(machineResults[i][0]),
                           np.array(machineResults[i][1]),
                           machineResults[i][2])

  finally:
    f.close()

  return (blobs, np.array(humanLabels), machineResults)
  
