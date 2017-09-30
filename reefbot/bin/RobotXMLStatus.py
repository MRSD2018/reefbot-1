'''Can return an XML document describing the change in state of the robot.

This uses a set of user defined modules subclassed from
RobotXMLStatus.Module that will define what type of robot state has
changed and could be returned in the XML.

Author: Mark Desnoyer (markd@cmu.edu)
Date: July 2010
'''
import roslib; roslib.load_manifest('reefbot')
import rospy
import json
import xml.dom.minidom
import time
import thread
import threading

class RobotXMLStatus:
  """This class serves as a clearing hosue for the state of the
  robot. As ROS messages arrive, this status is updated and then
  triggers are fired that cause the HTTP handler to return with the
  new data."""
  def __init__(self):
    self.modules = []

  def RegisterModule(self, module):
    self.modules.append(module)

  def GetXMLStateChange(self, lastTime):
    """Returns an xml document object specifying the changed state
    since lastTime."""
    impl = xml.dom.minidom.getDOMImplementation()
    doc = impl.createDocument("reefbot.com", "reefbot", None)
    root = doc.documentElement
    
    for module in self.modules:
      if module.HasChangedSince(lastTime):
        root.appendChild(module.GetXMLState(doc))

    return doc

  def GetJSONStateChange(self, lastTime):
    """Returns a JSON string specifying the changed states since lastTime."""
    outputState = []
    for module in self.modules:
      if module.HasChangedSince(lastTime):
        d = module.GetOutputDict()
        if d is None:
          rospy.logerr("Invalid change object from %s" % module)
        else:
          outputState.append(d)    
    return json.dumps(outputState)

  def HasChangedSince(self, lastTime):
    """Returns true if the Robot's status has changed since lastTime."""
    for module in self.modules:
      if module.HasChangedSince(lastTime):
        return True

    return False

class Module:
  """Abstract class that specifies the functions needed by a module
  that handles the RobotStatus. Modules that implement this
  interface should have ROS callbacks that are registered during the
  initialization"""
  
  def __init__(self, threadSignal):
    self.lastUpdate = time.time()
    self.threadSignal = threadSignal
    
  def GetXMLState(self, xmlDoc):
    """Returns a single xml Node defining the state of the module.
    
    xmlDoc - Document that we'll add to. This should only be used to
    call xmlDoc.createElement() and their ilk
    """
    raise NotImplemented("This is an abstract class")

  def GetOutputDict(self):
    """Returns a dictionary of the variables that we want to output.

    One of those entries must be 'name'
    """
    raise NotImplemented("This is an abstract class")

  def SignalUpdate(self):
    """The subclass should call this function to signal that an
    update has arrived."""
    self.threadSignal.acquire()
    self.lastUpdate = time.time()
    self.threadSignal.notifyAll()
    self.threadSignal.release()

  def HasChangedSince(self, someTime):
    """Returns true if this module has changed since someTime"""
    return self.lastUpdate > someTime

  def _CreateXMLValueNode(self, xmlDoc, name, value):
    node = xmlDoc.createElement(name)
    node.appendChild(xmlDoc.createTextNode(str(value)))
    return node
