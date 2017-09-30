import roslib; roslib.load_manifest('species_id')
import rospy
from species_id.msg import Parameter
from std_msgs.msg import String

class ParameterSetter:
  '''Controls the parameters set on the ROS server and removes them when the object dies.

  Also optionally prints out the parameters to a bag as Parameter messages'''

  def __init__(self, namespace, parameters, parameterBag=None, bagTopic='parameters', bagTime=None):
    '''Sets the parameters.

    namespace: Namespace for the parameters on the server
    parameters: a list of (paramName, paramValue) tuples
    parameterBag: bag object to print the parameters to
    bagTopic: topic to print the parameters on
    bagTime: ros Time object time in the bag to start print parameters
    '''
    self.namespace = namespace
    self.parameters = parameters
    # Grab rospy so that it is not garbage collected before this object dies
    self.rospy = rospy
    rospy.loginfo('Using parameters: ' + str(parameters))

    for paramName, paramValue in parameters:
      rospy.set_param('/%s/%s' % (namespace, paramName), paramValue)

    if parameterBag is not None:
      if bagTime is None:
        bagTime = rospy.Time.now()
      self.lastParameterTime = bagTime
      for paramName, paramValue in parameters:
        self.OutputParameterToBag(parameterBag, bagTopic, paramName, paramValue)

  def __del__(self):
    self.RemoveParameters()

  def RemoveParameters(self):
    for paramName, paramValue in self.parameters:
      try:
        self.rospy.delete_param('/%s/%s' % (self.namespace, paramName))
      except KeyError: pass
    self.parameters = []

  def OutputParameterToBag(self, bag, topic, name, val):
    nameMsg = String()
    nameMsg.data = name

    valMsg = String()
    valMsg.data = str(val)

    msg = Parameter()
    msg.name = nameMsg
    msg.value = valMsg

    bag.write(topic, msg, self.lastParameterTime)
    
    self.lastParameterTime += rospy.Duration.from_sec(0.01)
