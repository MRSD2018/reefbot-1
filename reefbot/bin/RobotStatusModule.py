'''Keeps track of the status of the robot as defined by the RobotStatus message

Author: Mark Desnoyer (markd@cmu.edu) Date: July 2010
'''
import rospy
from reefbot_msgs.msg import RobotStatus
import RobotXMLStatus

def callback(data, robotStatusModule):
  """Handler for the RobotStatus messages from ROS."""
  robotStatusModule.UpdateStateFromROS(data)

def fEq(a, b, tol=1e-5):
  '''Returns true if two floating point numbers are equal to a tolerance'''
  return abs(a - b) < tol

class RobotStatusModule(RobotXMLStatus.Module):  
  def __init__(self, threadSignal):
    RobotXMLStatus.Module.__init__(self, threadSignal)
    rospy.Subscriber(rospy.get_param('robot_status_topic', 'robot_status'),
                     RobotStatus, callback, self)
    self.leftSpeed = 0.0
    self.rightSpeed = 0.0
    self.vertSpeed = 0.0
    self.depth = 0.0
    self.heading = 0.0
    self.pitch = 0.0
    self.humidity = 0.0
    self.water_temp = 0.0

  def GetOutputDict(self):
    return { 'name' : 'RobotStatus',
             'left_speed' : self.leftSpeed,
             'right_speed' : self.rightSpeed,
             'vert_speed' : self.vertSpeed,
             'depth' : self.depth,
             'heading': self.heading,
             'pitch' : self.pitch,
             'humidity' : self.humidity,
             'water_temp' : self.water_temp}
    

  def UpdateStateFromROS(self, rosData):
    """Updates the robot status using the ROS message."""
    changed = False
    
    if not fEq(rosData.left_speed, self.leftSpeed):
      self.leftSpeed = rosData.left_speed
      changed = True

    if not fEq(rosData.right_speed, self.rightSpeed):
      self.rightSpeed = rosData.right_speed
      changed = True

    if not fEq(rosData.vertical_speed, self.vertSpeed):
      self.vertSpeed = rosData.vertical_speed
      changed = True

    if not fEq(rosData.depth, self.depth):
      self.depth = rosData.depth
      changed = True

    if not fEq(rosData.heading, self.heading):
      self.heading = rosData.heading
      changed = True

    if not fEq(rosData.pitch, self.pitch):
      self.pitch = rosData.pitch
      changed = True

    if not fEq(rosData.internal_humidity, self.humidity):
      self.humidity = rosData.internal_humidity
      changed = True

    if not fEq(rosData.water_temp, self.water_temp):
      self.water_temp = rosData.water_temp
      changed = True

    if changed:
      self.SignalUpdate()
    
