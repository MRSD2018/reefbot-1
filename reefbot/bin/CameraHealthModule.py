'''Keeps track of the camera health

Author: Mark Desnoyer (markd@cmu.edu)
Date: Sept 2010
'''
import rospy
from reefbot_msgs.msg import CameraHealth
import copy
import RobotXMLStatus

def callback(data, cameraHealthModule):
  """Handler for the RobotStatus messages from ROS."""
  cameraHealthModule.UpdateStateFromROS(data)

def fEq(a, b, tol=1e-5):
  '''Returns true if two floating point numbers are equal to a tolerance'''
  return abs(a - b) < tol

class CameraHealthModule(RobotXMLStatus.Module):  
  def __init__(self, threadSignal):
    RobotXMLStatus.Module.__init__(self, threadSignal)
    rospy.Subscriber(rospy.get_param('camera_health_topic', 'camera_health'),
                     CameraHealth, callback, self)
    self.lastRosMsg = CameraHealth(ping_ok=False, http_ping_ok=False,
                                   error_code=1)

  def GetOutputDict(self):
    return { 'name' : 'CameraHealth',
             'ping_ok' : self.lastRosMsg.ping_ok,
             'http_ping_ok' : self.lastRosMsg.http_ping_ok,
             'error_code' : self.lastRosMsg.error_code}
    

  def UpdateStateFromROS(self, rosData):
    """Updates the robot status using the ROS message."""
    changed = False

    for attrName in rosData.__slots__:
      if (attrName <> "header") and (
        getattr(self.lastRosMsg, attrName) <> getattr(rosData, attrName)):
        changed = True

    self.lastRosMsg = copy.deepcopy(rosData)
    
    if changed:
      self.SignalUpdate()
    
