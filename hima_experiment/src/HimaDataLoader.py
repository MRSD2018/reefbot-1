import roslib; roslib.load_manifest('hima_experiment')
import rospy
import os.path
import re
from sensor_msgs.msg import RegionOfInterest
from hima_experiment.msg import PersonLocation as PersonLocationMsg
from scipy import weave
from scipy.weave import converters
import cDetectionUtils

class PersonLocation:
  UNKNOWN = -1
  PARTIALLY_VISIBLE = 0
  ALL_VISIBLE = 1
  '''Hold for information about the location of a person.

  personId - Id of a person
  bbox - (x1, y1, x2, y2) bounding box around the person
  annotationType - -1 for unknown. 0 is partially visible and 1 is completely visible
  '''
  def __init__(self, personId, bbox, annotationType):
    self.id = personId
    self.bbox = bbox
    self.annotationType = annotationType

  def __eq__(self, other):
    return (self.id == other.id and
            self.bbox == other.bbox and
            self.annotationType == other.annotationType)

  def __ne__(self, other):
    return not self == other

  def Area(self):
    return (self.bbox[3] - self.bbox[1]) * (self.bbox[2] - self.bbox[0])

  def BuildMsg(self):
    bbox = RegionOfInterest()
    bbox.x_offset = self.bbox[0]
    bbox.y_offset = self.bbox[1]
    bbox.height = self.bbox[3] - self.bbox[1]
    bbox.width = self.bbox[2] - self.bbox[0]
    
    personLocation = PersonLocationMsg()
    personLocation.targetId = self.id
    personLocation.annotationType = self.annotationType
    personLocation.bbox = bbox

    return personLocation

  def CalculateOverlap(self, regionOfInterest):
    '''Calculates the overlap between this person\'s location and a RegionOfInterst message.'''
    return cDetectionUtils.CalculateOverlapOfRegions(
      (regionOfInterest.x_offset,
       regionOfInterest.y_offset,
       regionOfInterest.height,
       regionOfInterest.width),
      (self.bbox[0], self.bbox[1],
       self.bbox[3] - self.bbox[1],
       self.bbox[2] - self.bbox[0]))


  def CalculateOverlapTuple(self, tup):
    '''Calculate the overlap with a (x, y, height, width) tuple.'''
    return cDetectionUtils.CalculateOverlapOfRegions(
      tup,
      (self.bbox[0], self.bbox[1],
       self.bbox[3] - self.bbox[1],
       self.bbox[2] - self.bbox[0]))

  def CalculateOverlapTupleFast(self, tup):
    '''Calculate the overlap with a (x, y, height, width) tuple.'''
    boxX = self.bbox[0]
    boxY = self.bbox[1]
    boxH = self.bbox[3] - self.bbox[1]
    boxW = self.bbox[2] - self.bbox[0]
    tupX = tup[0]
    tupY = tup[1]
    tupH = tup[2]
    tupW = tup[3]

    code = '''
    return_val = 0.0;

    if (boxH > 0 && boxW > 0) {
      const double midYA = tupY + tupH/2.0;
      const double midYB = boxY + boxH/2.0;
      const double dRy = tupH/2.0 + boxH/2.0 - abs(midYA - midYB);

      const double midXA = tupX + tupW/2.0;
      const double midXB = boxX + boxW/2.0;
      const double dRx = tupW/2.0 + boxW/2.0 - abs(midXA - midXB);

      if (dRx > 1e-5 && dRy > 1e-5) {
        const double areaIntersection = dRy * dRx;
        const double areaUnion = (double)(tupH)*tupW +
                           (double)(boxH)*boxW -
                           areaIntersection;
        return_val = areaIntersection / areaUnion;
      }
      
    }'''

    overlap = weave.inline(code, ['boxX', 'boxY', 'boxH', 'boxW',
                                  'tupX', 'tupY', 'tupH', 'tupW',],
                           type_converters=converters.blitz,
                           compiler = 'gcc')
    return overlap
    
    
    

class FrameInfo:
  '''Holder for info about a frame.

  image - OpenCV image
  imageFile - Filename for the image
  timestamp - float of the timestamp when the frame was taken
  peopleLocations - Dictionary of id->PersonLocation objects representing the ground truth locations of the people in the scene
  '''
  def __init__(self, imageFile, timestamp, peopleLocations):
    self.imageFile = imageFile
    self.timestamp = timestamp
    self.peopleLocations = peopleLocations

  def GetMaxOverlap(self, regionOfInterest):
    '''Calculate the maximum percentage overlap between regionOfInterest and the people labeled in the image.

    regionOfInterest - A RegionOfInterest message
    '''
    overlap = [loc.CalculateOverlap(regionOfInterest) for personId, loc
               in self.peopleLocations.iteritems()]

    return max(overlap)

def ParseInputDirs(dirFilename):
  retval = []
  for line in open(dirFilename):
    retval.append(os.path.normpath(line.strip()))

  return retval

def LoadFrames(inputDir, leftDir, rightDir, annotationsFile,
               imgFileString='img_%04i.bmp'):
  '''Returns an array of FrameInfo objects.'''

  # Loop through the annotations building up the frame info
  frames = []
  curPeopleLocation = {}
  annotationsF = open(os.path.join(inputDir, annotationsFile))
  curFrameId = None
  timestamp = None
  for line in annotationsF:
    aSplit = line.strip().split()

    personId = int(aSplit[2])
    if personId < 0:
      # No person in this frame, so we don't need to create a location object
      continue
    
    frameId = int(aSplit[1])
    if frameId <> curFrameId:
      if curFrameId is not None:
        # Spit out the full info for the frame
        imageFile = os.path.join(inputDir, leftDir,
                                 imgFileString % curFrameId)
        frames.append(FrameInfo(imageFile, timestamp, curPeopleLocation))

      curPeopleLocation = {}
      
    curFrameId = frameId
    timestamp = float(aSplit[0])

    curPeopleLocation[personId] = PersonLocation(personId,
                                                 (int(float(aSplit[4])),
                                                  int(float(aSplit[5])),
                                                  int(float(aSplit[6])),
                                                  int(float(aSplit[7]))),
                                                 int(float(aSplit[3])))

  # Output the info for the last frame
  try:
    imageFile = os.path.join(inputDir, leftDir,
                             imgFileString % curFrameId)
    frames.append(FrameInfo(imageFile, timestamp, curPeopleLocation))
  except IOError as e:
    rospy.logerr('Error opening file: %s' % e)
  
  return frames

def LoadPeopleLocations(annotationFile, imageFiles):
  '''Creates a (imageFile,personId)->PersonLocation map from an annotation file and a list of image filenames.
  '''
  retval = {}

  rospy.loginfo('Reading annotation file: %s' % annotationFile)
  
  # First generate a map from the frame id to the image filename
  numberRe = re.compile('([0-9]+)\.((bmp)|(png)|(jpg))')
  id2fn = {}
  for imageFile in imageFiles:
    frameId = int(numberRe.search(imageFile).groups()[0])
    id2fn[frameId] = imageFile

  # Now parse the annotations file
  annotationF = open(annotationFile)
  try:
    for line in annotationF:
      aSplit = line.strip().split()
    
      frameId = int(aSplit[1])
      if frameId not in id2fn:
        # We don't care about this file
        continue
      
      personId = int(aSplit[2])
      if personId < 0:
        # No person in this frame, so we don't need to create a location
        # object
        continue

      retval[(id2fn[frameId],personId)] = PersonLocation(int(float(aSplit[2])),
                                                       (int(float(aSplit[4])),
                                                        int(float(aSplit[5])),
                                                        int(float(aSplit[6])),
                                                        int(float(aSplit[7]))),
                                                       int(float(aSplit[3])))

  finally:
    annotationF.close()

  return retval

    
