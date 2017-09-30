#!/usr/bin/python
'''
Takes an image capture stream and then farms out calls to other nodes
that try to find a type of species. It then takes the responses from
all those nodes and spits out a combined response.

The sub nodes can provide either a service with the definition in:
reefbot_msgs/FindSpecies

Or a sequence of topics that input a:
reefbot_msgs/ImageCapture
and output a
reefbot_msgs/SpeicesIDResponse

Input message: ImageCaptured
Output message: SpeciesIDResponse

Author: Mark Desnoyer
Date: May 2011
'''
import roslib; roslib.load_manifest('reefbot')
import rospy
import thread
import threading
from reefbot_msgs.msg import ImageCaptured
from reefbot_msgs.msg import SpeciesIDResponse
from reefbot_msgs.srv import FindSpecies

class SpeciesFinderRequest(threading.Thread):
  '''Abstract class for a request.'''
  def __init__(self, request, timeout, threadSignal):
    threading.Thread.__init__(self)
    self.timeout_ = timeout
    self.signal_ = threadSignal
    self.request_ = request
    self.answers = []
    self.hasAnswer = False
    self.didTimeout = False

  def GetAnswers(self):
    '''Returns the answers from the request or an empty list. This is blocking'''
    self.signal_.acquire()
    
    while not self.hasAnswer:
      self.signal_.wait()

    self.signal_.release()
      
    return self.answers

  def FlagTimeout(self):
    if not self.DidTimeout():
      self.signal_.acquire()
      self.hasAnswer = True
      self.didTimeout = True
      self.answers = []
      self.signal_.notifyAll()
      self.signal_.release()

  def DidTimeout(self):
    return self.didTimeout

class ServiceRequest(SpeciesFinderRequest):
  '''A request to a service.'''
  def __init__(self, request, timeout, threadSignal, serviceCall):
    SpeciesFinderRequest.__init__(self, request, timeout, threadSignal)
    self.serviceCall_ = serviceCall

  def run(self):
    if self.timeout_ is not None:
      timer = threading.Timer(self.timeout_, self.FlagTimeout)
      timer.start()

    try:
      response = self.serviceCall_(image=self.request_)
    except Exception as e:
      rospy.logwarn('Error calling service %s: %s' %
                    (self.serviceCall_.resolved_name, e))
      self.FlagTimeout()
      return
    rospy.logdebug('Response returned')
    self.signal_.acquire()
    if not self.DidTimeout():
      self.answers = response.response.answers
      self.hasAnswer = True
    self.signal_.notifyAll()
    self.signal_.release()

class PubSubRequest(SpeciesFinderRequest):
  '''A request to a publish subscribe loop.'''
  def __init__(self, request, timeout, threadSignal, publisher):
    SpeciesFinderRequest.__init__(self, request, timeout, threadSignal)
    self.publisher_ = publisher
    self.image_id_ = request.image_id
    if self.timeout_ is not None:
      self.timer = threading.Timer(self.timeout_, self.FlagTimeout)
      

  def run(self):
    self.signal_.acquire()
    self.publisher_.publish(self.request_)
    if self.timeout_ is not None:
      self.timer.start()
    self.signal_.release()

  def HandleResponse(self, response):
    if response.image_id <> self.image_id_:
      rospy.logerr('Image ids do not match %i <> %i' % (response.image_id,
                                                        self.image_id_))
      return
    
    self.signal_.acquire()
    if not self.DidTimeout():
      self.answers = response.answers
      self.hasAnswer = True
    self.signal_.notifyAll()
    self.signal_.release()

class SpeciesFinderPlugin:
  '''Abstract class for a species finder plugin.'''
  def __init__(self, namespace):
    self.namespace_ = namespace
    self.timeout_ = rospy.get_param('~timeout', 5)

  def SendRequest(self, request):
    '''Sends a request and returns a SpeciesFinderRequest object to handle the comms.'''
    raise NotImplementedError()

class ServiceSpeciesFinder(SpeciesFinderPlugin):
  '''Class to define looking for a species from a service call.'''
  def __init__(self, namespace):
    SpeciesFinderPlugin.__init__(self, namespace)
    self.serviceName_ = rospy.get_param('~%s/internal_service' % namespace)
    rospy.logdebug('Waiting for service to start')
    rospy.wait_for_service(self.serviceName_)
    self.service_ = rospy.ServiceProxy(self.serviceName_, FindSpecies)

  def SendRequest(self, request, threadSignal):
    rospy.logdebug('Sending request to %s' % self.serviceName_)
    retval = ServiceRequest(request, self.timeout_, threadSignal,
                            self.service_)
    retval.start()
    return retval

def PubSubSpeciesFinderCB(msg, obj):
  '''Callback function for the PubSubSpeciesFinder.'''
  obj.HandleResponse(msg)

class PubSubSpeciesFinder(SpeciesFinderPlugin):
  '''Class to define looking for a species using publishers and subscribers.'''
  def __init__(self, namespace):
    SpeciesFinderPlugin.__init__(self, namespace)
    self.requestTopic_ = rospy.get_param(
      '~%s/request_topic' % namespace)
    self.responseTopic_ = rospy.get_param(
      '~%s/response_topic' % namespace)
    
    self.publisher_ = rospy.Publisher(self.requestTopic_,
                                      ImageCaptured,
                                      tcp_nodelay=True)
    
    cb = lambda x: PubSubSpeciesFinderCB(x, self)
    rospy.Subscriber(self.responseTopic_,
                     SpeciesIDResponse,
                     cb)

    self.signal = threading.Semaphore()
    self.currentRequests = {}

  def SendRequest(self, request, threadSignal):
    retval = PubSubRequest(request, self.timeout_, threadSignal,
                           self.publisher_)
    
    self.signal.acquire()
    if request.image_id in self.currentRequests:
      rospy.logwarn('We already have an outgoing request for id: %i in namespace %s' % (msg.image_id, self.namespace_))
    self.currentRequests[request.image_id] = retval
    self.signal.release()
    
    retval.start()
    return retval

  def HandleResponse(self, msg):
    self.signal.acquire()
    try:
      curRequest = self.currentRequests[msg.image_id]
      del self.currentRequests[msg.image_id]
    except KeyError as e:
      rospy.logwarn('Did not have an outgoing request for id: %i in namespace %s' % (msg.image_id, self.namespace_))
      return
    finally:
      self.signal.release()

    curRequest.HandleResponse(msg)

def floatComp(x, y):
  diff = x-y
  if abs(diff) < 1e-5:
    return 0
  elif diff > 0:
    return 1
  else:
    return -1

mostRecentImageId = 0

def imageReceivedCallback(msg, plugins, publisher):
  global mostRecentImageId
  
  mostRecentImageId = msg.image_id

  # Send the requests
  responses = []
  threadSignal = threading.Condition()
  for plugin in plugins:
    responses.append(plugin.SendRequest(msg, threadSignal))

  # Collect all the responses
  combinedResponse = SpeciesIDResponse(image_id=msg.image_id)
  for response in responses:
    combinedResponse.answers.extend(response.GetAnswers())

  # Make sure that we're the most recent request
  if mostRecentImageId <> msg.image_id:
    return

  # Sort the answers by the best score
  combinedResponse.answers.sort(cmp=lambda x,y:
    floatComp(y.best_species[0].score, x.best_species[0].score))
  
  combinedResponse.header.stamp = rospy.Time.now()
  publisher.publish(combinedResponse)

if __name__ == '__main__':
  rospy.init_node('SpeciesFinderManager')

  # First build all the plugins. The plugins to use are specified by
  # the ROS param "plugin_list", which is a list of semi-color
  # separated tuples where each tuple is
  # <pluginType>,<pluginNamespace>. pluginType can be "Service" or
  # "PubSub" e.g. "Service,face_detector;PubSub,fish_finder"
  plugins = []
  pluginList = rospy.get_param('~plugin_list', '')
  for pluginString in pluginList.split(';'):
    (pluginType, pluginNamespace) = pluginString.split(',')
    plugins.append(eval('%sSpeciesFinder("%s")' % (pluginType, pluginNamespace)))

  publisher = rospy.Publisher(
    rospy.get_param('~species_id_response_topic', 'species_id'),
    SpeciesIDResponse,
    tcp_nodelay=True)

  # For each image that comes back, create a thread that will marshall
  # the results
  cb = lambda x: thread.start_new_thread(imageReceivedCallback,
                                         (x, plugins, publisher))
  rospy.Subscriber(rospy.get_param('~still_image_topic',
                                   'still_image'),
                   ImageCaptured,
                   cb)

  rospy.spin()
