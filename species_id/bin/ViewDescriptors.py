#!/usr/bin/python
'''Display in a plot the descriptors from some files.

PCA is done to reduce the descriptors to points in a reasonable number
of dimensions.

It takes in a list of files, one for each image of the format 1 produced by extract_features:

vector_dimension
nb_of_descriptors
x y a b c desc_1 desc_2 ......desc_vector_dimension
--------------------
where a(x-u)(x-u)+2b(x-u)(y-v)+c(y-v)(y-v)=1

Output is a file of the form:
vector_dimension
nb_of_visual_words
desc_1 desc_2 ......desc_vector_dimension
'''
usage = 'ViewDescriptors.py [<options>] -i <fileList>'

import optparse
from numpy import *
from pylab import *
import matplotlib.mlab as mlab
import os
import random
import sys
from mpl_toolkits.mplot3d import Axes3D

# Reopen stdout unbuffered
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)

def LoadDescFromFile(filename):
  '''Parses a file full of descriptors and return a dxm matrix of the m descriptors.'''
  print 'Openeing %s' % filename
  f = open(filename)
  try:
    d = int(f.readline())
    m = int(f.readline())
    fileDesc = empty((m,d), dtype=float)
    curEntry = 0
    for line in f:
      curVals = line.split()
      if len(curVals) == d:
        fileDesc[curEntry,:] = [float(x) for x in curVals]
      else:
        fileDesc[curEntry,:] = [float(x) for x in curVals[5:]]
      curEntry +=1
    if curEntry <> m:
      raise 'File is incomplete'
  finally:
    f.close()

  return fileDesc

def LoadDescriptors(filenames, prob=1.0):
  '''Loads a series of descriptors from an iterator of filenames'''
  retval = None
  for filename in filenames:
    curFileDesc =  LoadDescFromFile(filename.strip())
    n = curFileDesc.shape[0]
    if n == 0:
      continue

    # Do a random sampling of entries in this file
    nChoose = int(max(1, round(prob*n)))
    chosenIdx = random.sample(xrange(n), nChoose)
    curFileDesc = curFileDesc[chosenIdx, :]
    if retval is None:
      retval = curFileDesc
    else:
      retval = append(retval, curFileDesc, axis=0)
      
  return retval

def OutputDictionary(filename, words):
  '''Write a file containing the words in the dictionary.'''
  (k,d) = words.shape

  f = open(filename, 'w')
  try:
    f.write("%i\n%i\n" % (d,k))
    for i in range(k):
      f.write("%s\n" % (' '.join(['%f' % x for x in words[i,:]])))
  finally:
    f.close()

if __name__ == '__main__':
  # All of the command line flags
  parser = optparse.OptionParser()

  parser.add_option('-i', dest="inFile", default=None,
                    help="Input file specifying the filename of all the desc files")
  parser.add_option('--fraction_use', dest='fractionUse', default=1.0,
                    type='float', help='Fraction of descriptors to actually use in the nearest neighbour')
  parser.add_option('--rand_seed', dest='randSeed', default=165498, type='int',
                    help='Random seed')
  parser.add_option('--ndims', default=2, type='int',
                    help='Number of dimensions to display')

  (options, args) = parser.parse_args()

  random.seed(options.randSeed);

  # Load up the data
  f = open(options.inFile)
  try:
    dataset = LoadDescriptors(f, prob=options.fractionUse)
  finally:
    f.close()

  print 'Opened up %i datapoints' % dataset.shape[0]

  # Perform PCA
  #pcaObj = mlab.PCA(dataset)
  dataset = dataset.transpose()
  
  (pcaMat, pcaWeights, fracVar) = mlab.prepca(dataset)

  print 'The top fractions are: %s' % str(fracVar[0:10])

  if options.ndims == 2:
    figure(1)
    scatter(np.dot(pcaMat[:,0],dataset), np.dot(pcaMat[:,1], dataset))
  elif options.ndims == 3:
    fig = figure(1)
    ax = Axes3D(fig)
    ax.scatter(np.dot(pcaMat[:,0],dataset),
               np.dot(pcaMat[:,1],dataset),
               np.dot(pcaMat[:,2],dataset))

  show()
