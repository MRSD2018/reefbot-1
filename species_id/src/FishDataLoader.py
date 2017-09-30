'''Tools to load up data about the fish.'''
import os.path
import os

class FrameInfo:
  '''Information about a frame.'''
  def __init__(self, movieId, frameId, frameNum, imgFile):
    self.movieId = movieId
    self.frameId = frameId
    self.frameNum = frameNum
    self.imgFile = imgFile

def LoadFrameInfoFromBlobs(parsingRegex, blobDir):
  '''Returns a list of FrameInfo objects, one for each frame we could parse in the blobDir. The list is sorted by frame id'''
  frameList = []
  
  for curFile in os.listdir(blobDir):
    parse = parsingRegex.search(curFile)
    if parse:
      blobFile = open(os.path.join(blobDir, curFile))
      try:
        imgName = blobFile.readline().strip()
        if imgName[0] <> '/':
          imgName = os.path.normpath(os.path.join(blobDir, imgName))
        if not os.path.exists(imgName):
          raise IOError("Image file does not exist: " + imgName)
      
        frameList.append(FrameInfo(parse.groups()[1].strip('-'),
                                   parse.groups()[0],
                                   int(parse.groups()[3]),
                                   imgName))
      finally:
        blobFile.close()


  # Sort the list by frame id
  return sorted(frameList, key=lambda x: x.frameId)
                         
                                 
