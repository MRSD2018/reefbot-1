'''Handles when a still image is captures from the camera

Author: Mark Desnoyer (markd@cmu.edu)
Date: July 2010
'''
import roslib; roslib.load_manifest('reefbot')
import rospy
from reefbot_msgs.msg import ImageCaptured
from ImagePath import ImagePath
from cv_bridge import CvBridge
import RobotXMLStatus
import cv
import thread

def threaded_callback(data, stillCaptureModule):
  """Threaded version of the callback function"""
  thread.start_new_thread(callback, (data, stillCaptureModule))

def callback(data, stillCaptureModule):
  """Handler for the RobotStatus messages from ROS."""
  stillCaptureModule.UpdateStateFromROS(data)

class StillCaptureModule(RobotXMLStatus.Module):  
  def __init__(self, threadSignal):
    RobotXMLStatus.Module.__init__(self, threadSignal)
    rospy.Subscriber(rospy.get_param('still_image_topic', 'still_image'),
                     ImageCaptured, threaded_callback, self)
    self.lastImagePath = None
    self.bridge = CvBridge()
    self.path = ImagePath()

  def GetOutputDict(self):
    retVal = { 'name' : 'StillImage' }
    if not self.lastImagePath is None:
      retVal['path'] = self.lastImagePath;
    return retVal
    

  def UpdateStateFromROS(self, rosData):
    """Updates the robot status using the ROS message."""
    # We received an image, so build up the path for where to write it
    if not self.path.MakeImagePath():
      return
    osImagePath = self.path.GetOSImagePath(rosData.image_id);

    # Dump the image to disk
    try:
      cv_image = self.bridge.imgmsg_to_cv(rosData.image, "rgb8")
      cv.SaveImage(osImagePath, cv_image)
    except CvBridgeError as e:
      rospy.logerr("Could not convert to opencv image" + str(e))
      return
    except Exception as e:
      rospy.logerr("Error saving image to disk" + str(e))
      return    

    # The image is on disk, flag the UI
    self.lastImagePath = self.path.GetUIImagePath(rosData.image_id)
    rospy.logdebug("Signaling a new image at: " + self.lastImagePath)
    self.SignalUpdate()
    
