import roslib; roslib.load_manifest('species_id')
import rosbag
import rospy
from species_id import DetectionUtils
import numpy as np
import gzip
import hashlib
import struct
import cPickle as pickle
import cv_bridge
from cv_blobs import Blob
import cv2

VERSION='4'

class VUParse:
  def InitData(self):
    # For each candidate box, the scores and overlaps
    self.scores = []
    self.overlaps = []
  
    # --The following 3 items are in the same order--
    # For each human labeled fish, the maximum score that overlaps
    self.maxOverlapScore = [] 
    self.blobList = [] # list of (fileId, blobId)
    self.fishLoc = [] # list of (x,y,h,w) tuples of the ground truth for the fish
    
    self.fileMap = {} # filename -> fileId
    self.fileIdMap = {} # fileId -> filename
    self.params = {} # bagFile-> {name->value}
    self.processingTime = [] # list of all the processing times for each frame
    self.md5 = None # An identifier for this object

    # List of tuples specifying the first location for each bag in (scores, maxOverlapScore)
    self.bagIdx = []

    # Map from the fileId to the range in scores associated with that image
    self.fileRange = {}
  
  def __init__(self, inputBags, md5=None, frameSubsetRate=1, blobDir=None,
               overlapThresh=0.5, allowInvalidScores=False):
    self.InitData()
    self.md5 = md5
    blobReader = Blob.BlobSerializer()
    ndBridge = cv_bridge.NumpyBridge()

    curFileId = 0
    for bagFn in inputBags:
      rospy.loginfo('Reading bag ' + bagFn)
      curParams = {}

      curBag = rosbag.Bag(bagFn)
      self.bagIdx.append((len(self.scores), len(self.maxOverlapScore)))
      try:
        for topic, msg, t in curBag.read_messages(topics=['results']):
          if topic == 'parameters':
            # Store the parameter as strings
            curParams[msg.name.data] = msg.value.data
            
          if topic == 'results':
            if curFileId % frameSubsetRate <> 0:
              curFileId += 1
              continue

            rospy.loginfo('Processing ' + msg.image)
            blobFn = img2blobFn(msg.image, blobDir)
            self.fileIdMap[curFileId] = blobFn
            self.fileMap[blobFn] = curFileId
            self.processingTime.append(
              msg.scores.processing_time.data.to_sec())

            image = cv2.imread(msg.image, 0)

            scoreStartIdx = len(self.scores)

            blobStream = open(blobFn)
            try:
              blobs, garb = blobReader.Deserialize(blobStream, blobDir)

              regions = DetectionUtils.GenerateSamplingRegions(
                image.shape, msg.scores.grid)
              scoreGrid = ndBridge.matmsg_to_numpy(msg.scores.scores)
              validScores = np.nonzero(np.isfinite(scoreGrid))
              curOverlaps = overlaps
              if not allowInvalidScores:
                curOverlaps = overlaps[validScores]
              bestOverlaps = np.zeros((len(curOverlaps)))

              curBlobId = 0
              for blob in blobs:
                curLoc = (blob.minX, blob.minY, blob.maxY-blob.minY,
                          blob.maxX-blob.minX)
                self.blobList.append((curFileId, curBlobId))
                self.fishLoc.append(curLoc)

                overlaps = DetectionUtils.CalculateOverlapWithGrid(
                  curLoc, regions)

                # Get the maximum score that overlaps the fish
                validOverlaps = np.nonzero(overlaps > overlapThresh)
                if len(validOverlaps[0]) > 0:
                  self.maxOverlapScore.append(
                    np.amax(scoreGrid[validOverlaps]))
                else:
                  self.maxOverlapScore.append(float('-inf'))

                # Keep track of the maximum overlap for each location
                bestOverlaps = np.maximum(bestOverlaps, curOverlaps)

                curBlobId += 1

              # Store the scores and best overlaps for this file
              if allowInvalidScores:
                self.scores = np.hstack((self.scores, scoreGrid))
              else:
                self.scores = np.hstack((self.scores, scoreGrid[validScores]))
              
              self.overlaps = np.hstack((self.overlaps, bestOverlaps))

              # Keep track of the range where the scores are for this file
              self.fileRange[curFileId] = (scoreStartIdx, len(self.scores))

            finally:
              blobStream.close()

            curFileId += 1
            

      finally:
        curBag.close()
        self.params[bagFn] = curParams

    self.maxOverlapScore = np.array(self.maxOverlapScore)

def CreateVUParseObject(inputBags, options, cacheDir=None):
  # Determine the cache filename for the cached file
  hashVal = hashlib.md5()
  hashVal.update(VERSION)
  for bagFile in inputBags:
    hashVal.update(bagFile)
    hashVal.update(struct.pack('!d', os.path.getmtime(bagFile)))
  hashVal.update(struct.pack('!i', options.frame_subset_rate))
  hashVal.update(os.path.abspath(options.blob_dir))
  hashVal.update(struct.pack('!d', options.overlap_thresh))

  cacheFile = None
  if cacheDir is not None:
    cacheFile = os.path.join(cacheDir, '%s.vuparse' % hashVal.hexdigest())
    if os.path.exists(cacheFile):
      rospy.loginfo('Found cached VUParse object. Opening %s' % cacheFile)
      return pickle.load(gzip.open(cacheFile, 'rb'))

  parseObj = VUParse(inputBags, hashVal.hexdigest(),
                     options.frame_subset_rate,
                     options.blob_dir, options.overlap_thresh)

  if cacheDir is not None:
    outputFile = gzip.open(cacheFile, 'wb', 5)
    try:
      pickle.dump(parseObj, outputFile, 2)
    finally:
      outputFile.close()

  return parseObj
