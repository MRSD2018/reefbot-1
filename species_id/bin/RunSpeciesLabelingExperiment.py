#! /usr/bin/python
'''Runs an experiment that takes one video set vs. all for extracting the fish ID labels.'''
usage='RunSpeciesLabelingExperiment.py [options]'

import roslib; roslib.load_manifest('species_id')
import rospy
import csv
import copy
import numpy as np
from optparse import OptionParser
import os
import os.path
import random
import re
import subprocess
import string
import sys
import time
from species_id.srv import SpeciesID
import sensor_msgs.msg
import Blob

def ParseInputCSV(filename, videoRegexp, blobIdRegexp, blobRegexp):
  '''Parses the input csv so that each entry becomes a tuple of (blobFile, blobId, videoId, label).'''
  retVal = []

  f = open(filename)
  try:
    reader = csv.reader(f)
    # Skip first line
    reader.next()
    for line in reader:
      imageFile = line[0]
      label = line[1]
      tup = (blobRegexp.search(imageFile).groups()[0],
             int(blobIdRegexp.search(imageFile).groups()[0]),
             videoRegexp.search(imageFile).groups()[0],
             label)
      retVal.append(tup)

  finally:
    f.close()

  return retVal

def RunOneExperiment(fishEntries, blobDir, experimentDir, colorDict,
                     shapeDict, videoId=None, shapeWeight=0.5,
                     useSurfDescriptor=True,  useSiftDescriptor=False,
                     minBlobSize=0, fracTest=0.2,
                     resultFile = 'results.out',
                     useOpponentSurf=False, useCInvariantSurf=False,
                     queryBoxSize=None, saveIndex=False,
                     colorConverter='CV_BGR2HSV',
                     doGeometricRerank=False,
                     geoRerankThresh=0.0,
                     hessianThresh=400):
  # Split up the training and test entries
  if videoId is not None:
    trainEntries = filter(lambda x: x[2] <> videoId, fishEntries)
    testEntries = filter(lambda x: x[2] == videoId, fishEntries)
    rospy.loginfo('Testing with entries of video id ' + videoId)
  else:
    if fracTest < 0:
      trainEntries = fishEntries
      splitPoint = int(len(trainEntries) * 0.1)
      testEntries = trainEntries[0:splitPoint]
      rospy.loginfo('Testing with the training data')
    else:
      trainEntries = copy.deepcopy(fishEntries)
      random.shuffle(trainEntries)
      splitPoint = int(len(trainEntries) * fracTest)
      testEntries = trainEntries[0:splitPoint]
      trainEntries = trainEntries[splitPoint:]
      rospy.loginfo('Testing with random entries')
    videoId = ''

  runName = ''.join(random.choice(string.letters) for i in xrange(10))
  rospy.loginfo('Service name for this run is ' + runName)
    

  # Now build up an index using the training entries by running the
  # BuildFishIndex program
  labelFilename = os.path.join(experimentDir, 'train_%s.label' % runName)
  labelFile = open(labelFilename, 'w')
  try:
    for entry in trainEntries:
      labelFile.write('%s,%s,%s\n' % (entry[0], entry[1], entry[3]))
  finally:
    labelFile.close()

  # Let's be consistent with feature parameters
  featureParams = [
    ['color_dict_filename', colorDict],
    ['color_converter', colorConverter],
    ['color_frac', '0.1'],
    ['shape_dict_filename', shapeDict],
    ['surf_detector', True],
    ['surf_hessian_threshold', hessianThresh],
    ['surf_octaves', '3'],
    ['surf_octave_layers', '4'],
    ['surf_extended', False],
    ['surf_descriptor', useSurfDescriptor],
    ['sift_descriptor', useSiftDescriptor],
    ['shape_weight', str(shapeWeight)],
    ['min_shape_val', '0.01'],
    ['min_color_val', '0.01'],
    ['min_score', '0.01'],
    ['opponent_color_surf', useOpponentSurf],
    ['cinvariant_color_surf', useCInvariantSurf],
    ['use_bounding_box', queryBoxSize is not None],
    ['bounding_box_expansion',
     queryBoxSize if queryBoxSize is not None else 1.0],
    ['geometric_rerank', doGeometricRerank],
    ['geo_rerank_inlier_thresh', geoRerankThresh]]
  rospy.loginfo('Using parameters: ' + str(featureParams))

  # Set the parameters on the ROS server
  for featureName, value in featureParams:
    rospy.set_param('/%s/%s' % (runName,featureName), value)
  
  indexFilename = os.path.join(experimentDir, 'train_%s.index' % runName)
  rospy.set_param('/%s/index_file' % runName, indexFilename)

  try:
    retval = 1
    try:
      procSequence  = ['bin/BuildFishIndex', '--input', labelFilename,
                       '--output', indexFilename,
                       '--blob_prefix', blobDir + '/'] + \
                       ['--%s=%s' % (x[0], x[1]) for x in featureParams]
      rospy.loginfo("Running %s", procSequence)
      retval = subprocess.call(procSequence)

    finally:
      if retval <> 0:
        raise Exception("Could not build the fish index. Return code: %i\n %s" %
                        (retval, ' '.join(procSequence)))

    # Finally, start up a SpeciesIDNode to serve responses using the new
    # index
    serviceName = '/%s/species_id_service' % runName
    proc = subprocess.Popen(['bin/SpeciesIDRosNode',
                             '__name:=' + runName,
                             '_species_id_service:=' + serviceName])

    rospy.loginfo('Waiting for ROS node to be created')

    # Now setup a connection to the service that handles the SpeciesIDNode
    # TODO(mdesnoyer): make this a persistent connection
    rospy.wait_for_service(serviceName, timeout=6000)
    speciesIdService = rospy.ServiceProxy(serviceName, SpeciesID,
                                          persistent=False)
    try:
      results = []

      N_RESULTS = 5
    
      # Send requests to the service to get results
      for (blobFile, blobId, video_id, label) in testEntries:
        # TODO(mdesnoyer): Speed this up so that we only send one request for each image
        request = Blob.OpenBlobAsSpeciesIDRequest(os.path.join(blobDir,blobFile))
        request.regions = [request.regions[blobId]]

        # If the blob is too small, don't send a request
        blobSize = request.regions[0].bounding_box.height * \
                   request.regions[0].bounding_box.width
        if blobSize < minBlobSize:
          continue

        # If we're sending request that are just bounding boxes,
        # remove the mask and expand the bounding box as necessary.
        if queryBoxSize is not None:
          request.regions[0].mask = sensor_msgs.msg.Image()
          width = request.regions[0].bounding_box.width
          height = request.regions[0].bounding_box.height
          centerX = request.regions[0].bounding_box.x_offset + width/2
          centerY = request.regions[0].bounding_box.y_offset + height/2
          width = width * queryBoxSize
          height = height * queryBoxSize
          minX = int(round(max(0, centerX - width/2)))
          maxX = int(round(min(centerX + width/2, request.image.width-1)))
          minY = int(round(max(0, centerY - height/2)))
          maxY = int(round(min(centerY + height/2, request.image.height-1)))
          request.regions[0].bounding_box.x_offset = minX
          request.regions[0].bounding_box.y_offset = minY
          request.regions[0].bounding_box.width = maxX - minX
          request.regions[0].bounding_box.height = maxY - minY
      
      
        response = speciesIdService(request).response
        
        result = [blobFile, blobId, int(label)]
        for score in response.answers[0].best_species[0:N_RESULTS]:
          result.extend([score.species_id, score.score, score.meta_data])
        for i in range(N_RESULTS - len(response.answers[0].best_species)):
          result.extend([-1, -1, -1])
        results.append(result)
    
    finally:
      speciesIdService.close()
      proc.terminate()

  finally:
    # Delete the parameters on the ROS server
    rospy.delete_param('/%s/index_file' % runName)
    for featureName, value in featureParams:
      try:
        rospy.delete_param('/%s/%s' % (runName,featureName))
      except KeyError: pass

    # Delete the index
    if not saveIndex:
      os.remove(indexFilename)

  # Now dump the results to a CSV file for later analysis
  resultCSV = open(os.path.join(experimentDir, resultFile), 'wb')
  try:
    csvWriter = csv.writer(resultCSV)
    csvWriter.writerow(['File', 'BlobID', 'HumanLabel'] +
                       reduce(lambda x, y: x+y, 
                              [['SpeciesLabel%i' % i, 'Score%i' % i,
                                'MetaData%i' % i] for i in range(N_RESULTS)]))
    csvWriter.writerows(results)
    
  finally:
    resultCSV.close()

  # Calculate the accuracy
  if len(results) > 0:
    results = np.array([x[1:] for x in results])
    accuracy = np.sum(results[:,1] == results[:,2]) / float(results.shape[0])
    rospy.loginfo("Accuracy for leaving out video %s is %f" % (videoId, accuracy))
  

if __name__ == '__main__':
  # All of the command line flags
  parser = OptionParser(usage=usage)

  parser.add_option('--blob_dir', dest='blob_dir',
                    help='Directory where the blob files reside',
                    default=None)
  parser.add_option('--experiment_dir', dest='experiment_dir',
                    help='Directory to put the files used in the experiment and the results',
                    default='.')
  parser.add_option('--video_regexp', dest='video_regexp',
                    help='Regular expression used to extract the video id from the filename',
                    default='Fish101909-([0-9]+)-')
  parser.add_option('--image_list', dest='image_list',
                    help='CSV file of <image_filename>,<label> listing all the entries')
  parser.add_option('--blobid_regexp', dest='blobid_regexp',
                    help='Regular expression used to extract the blob id from the filename',
                    default='blob\.([0-9]+)\.')
  parser.add_option('--blob_regexp', dest='blob_regexp',
                    help='Regular expression used to extract the blob filename from the image filename',
                    default='(.+\.blob)')
  parser.add_option('--color_dict_filename',
                    help='Filename of the color dictionary',
                    default='/data/mdesnoye/fish/extractedFish/blob090710/hsv.dict')
  parser.add_option('--color_converter',
                    help='cvtColor string for color conversion',
                    default='CV_BGR2HSV')
  parser.add_option('--shape_dict_filename',
                    help='Filename of the shape dictionary',
                    default='/data/mdesnoye/fish/extractedFish/blob090710/surf.dict')
  parser.add_option('--use_surf_descriptor', action='store_true',
                    default=False, help='Use SURF descriptors?')
  parser.add_option('--use_sift_descriptor', action='store_true',
                    default=False, help='Use SIFT descriptors?')
  parser.add_option('--use_opponent_surf', action='store_true',
                    default=False, help='Use Opoonent color SURF descriptors?')
  parser.add_option('--use_cinvariant_surf', action='store_true',
                    default=False, help='Use C-Invariant SURF descriptors?')
  parser.add_option('--video_ids', default=None,
                    help='Python code specifying which video ids to use')
  parser.add_option('--shape_weights', default='[0.5]',
                    help='Python code specifying an iterable of shape weights to test')
  parser.add_option('--min_blob_size', default='[0]', 
                    help='Python code specifying an iterable of blob sizes to test')
  parser.add_option('--hess_thresh', default=400, type="float",
                    help='Python code specifying an iterable of different hessian thresholds to test')
  parser.add_option('--frac_test', default=0.2, type="float",
                    help="Fraction of entries to use for testing when doing random distribution")
  parser.add_option('--query_box_size', default='[None]',
                    help='Python code for specifying the box sizes to query with. This is for sending requests using bounding boxes that get progressively bigger around the target')
  parser.add_option('--save_index', action='store_true', default=False,
                    help="Save the index that was built")
  parser.add_option('--do_geo_rerank', action='store_true', default=False,
                    help="Do geometric reranking?")
  parser.add_option('--geo_rerank_inlier_thresh', default='[3.0]',
                    help="Distance in pixels to consider a point to be an inlier when doing geometric reranking")

  (options, args) = parser.parse_args()

  rospy.init_node('SpeciesLabelingExperiment', anonymous=True)

  fishEntries = ParseInputCSV(options.image_list,
                              re.compile(options.video_regexp),
                              re.compile(options.blobid_regexp),
                              re.compile(options.blob_regexp))

  # Get the list of unique video ids
  if options.video_ids is None:
    videoIds = set([x[2] for x in fishEntries])
  else:
    videoIds = eval(options.video_ids)

  

  # Run each of the experiments
  for videoId in videoIds:
    for shapeWeight in eval(options.shape_weights):
      for minBlobSize in eval(options.min_blob_size):
        for queryBoxSize in eval(options.query_box_size):
          for geoRerankThresh in eval(options.geo_rerank_inlier_thresh):
            RunOneExperiment(fishEntries, options.blob_dir,
                             options.experiment_dir,
                             options.color_dict_filename,
                             options.shape_dict_filename,
                             videoId=videoId,
                             shapeWeight=shapeWeight,
                             useSurfDescriptor=options.use_surf_descriptor,
                             useSiftDescriptor=options.use_sift_descriptor,
                             minBlobSize=(minBlobSize*minBlobSize),
                             fracTest = options.frac_test,
                             resultFile=('results_shw%s_video%s_blob%i_bbox%s_geo%i.csv' %
                                         (shapeWeight, videoId, minBlobSize,
                                          queryBoxSize, geoRerankThresh)),
                             useOpponentSurf=options.use_opponent_surf,
                             useCInvariantSurf=options.use_cinvariant_surf,
                             queryBoxSize=queryBoxSize,
                             saveIndex=options.save_index,
                             colorConverter=options.color_converter,
                             doGeometricRerank=options.do_geo_rerank,
                             geoRerankThresh=geoRerankThresh,
                             hessianThresh=options.hess_thresh)
