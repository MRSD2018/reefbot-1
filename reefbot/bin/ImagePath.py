'''Utility module to determine the path to the images.

Author: Mark Desnoyer (markd@cmu.edu)
Date: Aug 2010
'''
import rospy
import datetime
import os
import os.path

class ImagePath:
  def __init__(self, imageDir=None, htmlDir=None):  
    # Directory to save the images to. Relative to the path where the
    # html file will be served from.
    if imageDir is None:
      self.imageDir = rospy.get_param('image_dir', 'captured')
    else:
      self.imageDir = imageDir

    if htmlDir is None:
      self.htmlDir = rospy.get_param('html_dir',
                                     '/home/mdesnoyer/src/reefbot/webui')
    else:
      self.htmlDir = htmlDir

  def GetOSImagePath(self, image_id):
    '''Return the path to the image specified by image_id.'''
    return '%s/%s' % (self.htmlDir, self.GetUIImagePath(image_id))

  def GetUIImagePath(self, image_id):
    '''Return the path to the image specified by image_id relative to the html directory.'''
    day = datetime.date.today().strftime("%Y%m%d")
    return '%s/%s/%010d.jpg' % (self.imageDir, day, image_id)

  def MakeImagePath(self):
    '''Builds the path to the images if it\'s not there.

    return True on sucessful build
    '''
    curDir = '/'.join([self.htmlDir,
                       self.imageDir,
                       datetime.date.today().strftime("%Y%m%d")])
    if not os.path.exists(curDir):
      try:
        os.makedirs(curDir)
      except OSError as e:
        rospy.logerr("Could not create directory for images: %s" % str(e))
        return False
    return True
  
