#!/usr/bin/python
'''Converts an xml from the mechanical turk annotation system into a blob file.'''
usage='xml2blob.py [options]'

import os
import os.path
import sys
from optparse import OptionParser
import xml.dom.minidom
import re
import numpy as np
import Blob

def ConvertCoords(imgW, imgH, x, y):
  newX = max(imgW, imgH)/ 500. * x - max(0, (imgH-imgW)/2)
  newY = max(imgW, imgH)/ 500. * y - max(0, (imgW-imgH)/2)
  return (newX, newY)

def ScaleDim(imgW, imgH, dim):
  return max(imgW, imgH)/500. * dim

def ParseXML(filename):
  '''Parses the XML.

  Coordinates are in the image frame

  Returns:
  Tuple of (imgURL, imgShape, [(left, top, width, height)])
  '''
  dom = xml.dom.minidom.parse(filename)

  # First find out the width and heights of the image
  width = float(dom.getElementsByTagName("width")[0].childNodes[0].data)
  height = float(dom.getElementsByTagName("height")[0].childNodes[0].data)

  # Get the image url
  imgNode = dom.getElementsByTagName("image")[0]
  imgURL = imgNode.getAttribute("url")

  # Now if there are no fish or more than 10, we're done
  for binaryNode in dom.getElementsByTagName("attribute"):
    if binaryNode.getAttribute('value') == "1":
      return (imgURL, (height, width), [])

  # Parse the bounding boxes
  bboxList = []
  for bboxNode in dom.getElementsByTagName("bbox"):
    bboxList.append((
      ConvertCoords(width, height,
                    float(bboxNode.getAttribute('left')), 0)[0],
      ConvertCoords(width, height,
                    0, float(bboxNode.getAttribute('top')))[1],
      ScaleDim(width, height,
               float(bboxNode.getAttribute('width'))),
      ScaleDim(width, height,
               float(bboxNode.getAttribute('height')))))

  return (imgURL, (height, width), bboxList)

class FrameIdConverter:
  def __init__(self, dictFile):
    self.ids = {}
    if dictFile is not None:
      # Parse the dictionary file
      for line in open(dictFile):
        oldId, newId = line.strip().split(',')
        self.ids[oldId] = newId

  def ToNewId(self, oldId):
    if len(self.ids) == 0:
      return oldId
    
    try:
      return self.ids[oldId]
    except KeyError as e:
      print 'Warning: Could not find old id: %s' % oldId
      raise e
    

if __name__ == '__main__':
  # All of the command line flags
  parser = OptionParser(usage=usage)

  parser.add_option('--input', '-i', dest="input",
                    help='Filename listing the xml files one per line',
                    default=None)
  parser.add_option('--output_dir', 
                    help='Directory to output the blob files and label file',
                    default=None)
  parser.add_option('--frame_dir', 
                    help='Directory containing the frames',
                    default='/data/mdesnoye/fish/tank_videos/extracted_fish/20110102/frames/')
  parser.add_option('--frame_regex',
                    default='(([0-9][0-9]-){3}[0-9]+)\.',
                    help='Regex to extract the frame id from filenames')
  parser.add_option('--known_blob_dir', default=None,
                    help='Directory that contains the known blobs')
  parser.add_option('--known_labels', default=None,
                    help='File containing the known labels for the blobs in known_blob_dir. Each line is <filename>,<label>')
  parser.add_option('--blob_regex',
                    default='blob\.([0-9]+)\.',
                    help='Regex to extract the blob id from a filename')
  parser.add_option('--min_overlap', default=10, type='int',
                    help='Minimum number of pixels overlap to transfer the label')
  parser.add_option('--frameid_dict', default=None,
                    help='File that has fameid conversions in CSV format, one per line. <oldId>,<newId>')
  parser.add_option('--do_xml_old_ids', default=False,
                    action='store_true',
                    help='Use flag if the xml file contains old frame ids')
  (options, args) = parser.parse_args()

  frameRegex = re.compile(options.frame_regex)
  blobRegex = re.compile(options.blob_regex)
  blobSerializer = Blob.BlobSerializer()

  idConverter = FrameIdConverter(options.frameid_dict)

  # Load in the known labels
  knownLabels = {}
  for line in open(options.known_labels):
    f, label = line.strip().split(',')
    try:
      frameId = idConverter.ToNewId(frameRegex.search(f).groups()[0])
      blobId = int(blobRegex.search(f).groups()[0])
      knownLabels[(frameId, blobId)] = label
    except KeyError:
      pass

  # Create the output directory
  try:
    os.makedirs(options.output_dir)
  except os.error:
    pass

  # Open the files to take the output labels and the list of 
  outputLabelFile = open(os.path.join(options.output_dir,'imgLabels.txt'),
                         'w')
  try:

    # Go through all the xml files
    for line in open(options.input):
      xmlFile = line.strip()
    
      # Parse the xml file
      imgURL, imgShape, bboxList = ParseXML(xmlFile)
      frameId = frameRegex.search(imgURL).groups()[0]
      oldFrameId = frameId
      if options.do_xml_old_ids:
        try:
          frameId = idConverter.ToNewId(frameId)
        except KeyError:
          continue

      if len(bboxList) == 0:
        continue

      # Open up the known blob file for the frame
      knownBlobFn = os.path.join(options.known_blob_dir,
                                 '%s.blob' % oldFrameId)
      knownBlobs, frameFn = blobSerializer.Deserialize(open(knownBlobFn),
                                                       options.known_blob_dir)

      # Create the list of human labeled blobs
      newBlobs = Blob.BlobResult()
      for box in bboxList:
        newBlob = Blob.Blob()
        newBlob.AddBox(max(0, box[0]),
                       max(0, box[1]),
                       min(round(box[2]+box[0]), imgShape[1]) - 1,
                       min(round(box[3]+box[1]), imgShape[0]) - 1)
        newBlobs.AppendBlob(newBlob)

      # Dump the new blob to a file
      newBlobFile = open(os.path.join(options.output_dir,
                                       '%s.blob' % frameId), 'w')
      try:
        blobSerializer.Serialize(newBlobFile,
                                 newBlobs,
                                 os.path.join('..', 'frames',
                                               '%s.jpg' % frameId))
      finally:
        newBlobFile.close()

      # Match blobs and transfer labels
      for j in range(newBlobs.nBlobs()):
        newBlob = newBlobs.GetBlob(j).ToBinaryImageMask(imgShape)

        maxOverlap = 0
        bestLabel = None
        for i in range(knownBlobs.nBlobs()):
          # see if we have a label
          if (frameId, i) not in knownLabels:
            continue
      
          curKnownBlob = knownBlobs.GetBlob(i).ToBinaryImageMask(imgShape)

          # See if the two blobs match enough
          overlap = np.sum(np.bitwise_and(newBlob, curKnownBlob))
          if (overlap > maxOverlap):
            maxOverlap = overlap
            bestLabel = knownLabels[(frameId, i)]

        if maxOverlap > options.min_overlap and bestLabel is not None:
          outputLabelFile.write('%s.blob.%i.jpg,%s\n' %
                                (frameId, j, bestLabel))

  finally:
    outputLabelFile.close()
          
        

        
