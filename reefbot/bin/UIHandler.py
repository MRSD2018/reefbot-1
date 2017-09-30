#!/usr/bin/env python
'''This webserver will handle the requests for the user interface. We
use a long polling technique that only returns the results from a
request if something has changed in the robot state.

Author: Mark Desnoyer (markd@cmu.edu)
Date: July 2010
'''
import roslib; roslib.load_manifest('reefbot')
import rospy
import xml.dom.minidom
import re
import SocketServer
import string
import time
import thread
import threading
import urlparse
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
from RobotXMLStatus import RobotXMLStatus
from RobotStatusModule import RobotStatusModule
from VideoStreamModule import VideoStreamModule
from StillCaptureModule import StillCaptureModule
from CameraHealthModule import CameraHealthModule
from SpeciesSelectionModule import SpeciesSelectionModule

threadSignal = threading.Condition()
robotStatus = RobotXMLStatus()

class ThreadedPushServer(SocketServer.ThreadingMixIn, HTTPServer):
  daemon_threads = True
  
  def __init__(self, server_address, handler):
    HTTPServer.__init__(self, server_address, handler)
    self.clientDict = {}

  def GetLastRequestTime(self, clientAddress, _id):
    """Returns the last time that this client received a request."""
    if (clientAddress[0], _id) in self.clientDict:
      return self.clientDict[(clientAddress[0], _id)]
    else:
      return 0

  def SetRequestTime(self, clientAddress, _id):
    """Log that a request was sent to this client now"""
    self.clientDict[(clientAddress[0], _id)] = time.time()

class UIHandler(BaseHTTPRequestHandler):
  """Handles requests from the web for data. This server practices
  long polling so it will only respond to the request when data is
  new. Thus, it keeps track of when the last time it responded to each
  client."""
  def __init__(self,request,clientAddress, server):
    BaseHTTPRequestHandler.__init__(self,request,clientAddress,server)

  def getID(self):
    '''Determines the id of the UI based on the query string'''
    parsed = urlparse.urlparse(self.path)
    query = urlparse.parse_qs(parsed.query)
    if 'id' in query:
      return query['id'][0]
    else:
      return None

  def parseURL(self):
    parsed = urlparse.urlparse(self.path)
    query = urlparse.parse_qs(parsed.query)

    # Extract the ID
    if 'id' in query:
      self.id = query['id'][0]
    else:
      self.id =  None

    # Extract the name of the callback function
    self.callback = None
    if 'cb' in query:
      cbStr = query['cb'][0]
      if re.match(r'^[\w]+$', cbStr):
        self.callback = cbStr


  def do_GET(self):
    self.parseURL()

    if self.callback is None:
      self.send_response(400)
      self.end_headers()
      return
    
    lastTime = self.server.GetLastRequestTime(self.client_address, self.id)
    
    threadSignal.acquire()
    
    # Wait for the robot status to change
    while not robotStatus.HasChangedSince(lastTime):
      threadSignal.wait()

    try:
      jsonResponse = robotStatus.GetJSONStateChange(lastTime)

      # Send the response
      self.send_response(200) # if things aren't okay, don't send 200.
      self.send_header("Content-type","application/json; charset=utf-8")
      self.send_header("Cache-Control", "no-cache")
      self.send_header("Expires", "-1")
      self.end_headers()
      self.wfile.write("%s(%s)" % (self.callback, jsonResponse))

      # Do necessary bookeeping
      self.server.SetRequestTime(self.client_address, self.id)
    except IOError:
      # The client probably hung up
      pass
    
    finally:
      threadSignal.release()

  def log_error(self, fmt, *args, **kwargs):
    rospy.logerr(fmt % args)

  def log_message(self, fmt, *args, **kwargs):
    rospy.logdebug(fmt % args)
  
def run():
  rospy.init_node('UIHandler')

  # Define the modules
  modulesToAdd = [
    RobotStatusModule,
    VideoStreamModule,
    StillCaptureModule,
    CameraHealthModule,
    SpeciesSelectionModule]
  for moduleClass in modulesToAdd:
    try:
      module = moduleClass(threadSignal)
      robotStatus.RegisterModule(module)
    except Exception as e:
      rospy.logerr('Error initializing module %s: %s' % (module, e))

  # Start thread to listen for ROS events
  thread.start_new_thread(rospy.spin, ())

  # Turn on the HTTP server
  server_address = ('localhost',8000)
  httpd = ThreadedPushServer(server_address, UIHandler)
  try:
    httpd.serve_forever()
  finally:
    httpd.socket.close()

if __name__=="__main__":
  run()
  
