'''Can return an XML document describing the live video stream for the camera.

Author: Mark Desnoyer (markd@cmu.edu)
Date: July 2010
'''
import rospy
from reefbot_msgs.msg import VideoStream
import RobotXMLStatus

def callback(data, videoStreamModule):
  """Handler for the VideoStream messages from ROS."""
  videoStreamModule.UpdateStateFromROS(data)

class VideoStreamModule(RobotXMLStatus.Module):  
  def __init__(self, threadSignal):
    RobotXMLStatus.Module.__init__(self, threadSignal)
    rospy.Subscriber(rospy.get_param('video_stream_topic', 'video_stream'),
                     VideoStream, callback, self)
    self.url = rospy.get_param('video_url', '')

  def GetXMLState(self, xmlDoc):
    """Returns a single xml Node defining the state of the module.
    
    xmlDoc - Document that we'll add to. This should only be used to
    call xmlDoc.createElement() and their ilk
    """
    root = xmlDoc.createElement('VideoStream')

    root.appendChild(self._CreateXMLValueNode(xmlDoc, 'url',
                                              self.url))

    return root

  def GetOutputDict(self):
    return { 'name' : 'VideoStream',
             'url' : self.url }
    

  def UpdateStateFromROS(self, rosData):
    """Updates the robot status using the ROS message."""

    self.url = rosData.url.data

    # Always signal an update because this should be rare
    self.SignalUpdate()
