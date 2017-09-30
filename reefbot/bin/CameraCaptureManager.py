#!/usr/bin/python
'''The Camera Capture Manager that recieves joystick commands to take
a picture and then, grabs the picture from the camera publishes a
SpeciesID Request to figure out what fish was in the picture.

Author: Mark Desnoyer (markd@cmu.edu)
Date: July 2010
'''
import roslib; roslib.load_manifest('reefbot')
import rospy
import httplib
import sys
import time
import urllib
import ImageFile
import cv
import socket
from cv_bridge import CvBridge
from joy.msg import Joy
from reefbot_msgs.msg import ImageCaptured

class CameraCaptureManager:
  '''Class that handles capturing an image from the camera.'''
  def __init__(self):
    # ----------------------------#
    # Parameters for the process that are filled from the ros parameter server
    
    # IP address of the camera
    self.cameraIp = rospy.get_param("camera_ip", "192.168.1.13")

    # Joystick topic to listen to
    self.joystickTopic = rospy.get_param("joystick_topic", "joy")

    # Button index for the button that is used to signal that a picture
    # should be taken.
    self.buttonId = rospy.get_param("~button_id", 0)

    # Topic to publish the images on
    self.imageTopic = rospy.get_param("still_image_topic", "still_image")

    # Resolution of the camera. Can be "half" or "full"
    self.res = rospy.get_param("~res", "full")

    # Specifies the bounds of the requested image window. They cannot
    # exceed the size of the image sensor array and should be divisible by
    # 16 if Res is full and 32 if res is half.
    self.x0 = rospy.get_param("~x0", 352)
    self.y0 = rospy.get_param("~y0", 416)
    self.x1 = rospy.get_param("~x1", 3296)
    self.y1 = rospy.get_param("~y1", 2336)

    # JPEG quality with a range from 1 to 20
    self.quality = rospy.get_param("~quality", 10);

    #------------------------------#

    # State machine variables
    self.curImageId = long(rospy.Time.now().secs)
    self.buttonWasPressed = False

    # Other variables
    self.imagePublisher = None
    self.cvBridge = CvBridge()

  def __del__(self):
    pass

  def Init(self):
    self.imagePublisher = rospy.Publisher(self.imageTopic, ImageCaptured,
                                          tcp_nodelay=True, latch=False);
    rospy.Subscriber(self.joystickTopic, Joy, JoystickCallback, self)
    rospy.loginfo("Initialized Camera Capture Manager")

  def ConnectToCamera(self):
    rospy.loginfo("Connecting to camera at: %s" % self.cameraIp)
    cameraConnection = None
    connected = False
    while not connected:
      try:
        cameraConnection = httplib.HTTPConnection(self.cameraIp)
        cameraConnection.connect()
        connected = True
      except httplib.HTTPException as e:
        rospy.logerr('Cannot connect to the camera: %s' % e)
        rospy.sleep(5)
      except socket.error as e:
        rospy.logerr('Cannot connect to the camera: %s' % e)
        rospy.sleep(5)
    rospy.loginfo("Connected to camera at: %s" % self.cameraIp)
    return cameraConnection

  def RetrieveImageFromCamera(self):
    cameraConnection = self.ConnectToCamera()
    try:
      image = None

      # Define the settings for the camera
      params = urllib.urlencode({'res': self.res, 'x0': self.x0,
                                 'y0' : self.y0, 'x1' : self.x1,
                                 'y1' : self.y1, 'quality' : self.quality,
                                 'doublescan' : 1})
      # Request an image from the camera
      try:
        cameraConnection.request("GET", "/image?%s" % params)
        #cameraConnection.request("GET", "/h264f?res=full&x0=640&x1=1280&y0=352&y1=768&qp=16&doublescan=1&ssn=33&iframe=1")
        response = cameraConnection.getresponse()
      except httplib.HTTPException as e:
        rospy.logerr('Cannot connect to the camera: %s' % e)
        return None
      except socket.error as e:
        rospy.logerr('Cannot connect to the camera: %s' % e)
        return None

      if response.status != 200:
        # There was an error reading from the camera
        rospy.logerr('Received an error code from the camera %i, %s' %
                     (response.status, response.reason))
      else:    
        # We have a response from the camera so parse it out into a
        # message format
        parser = ImageFile.Parser()
        rawBytes = response.read(response.getheader('content-length'))
        parser.feed(rawBytes)
        pilImage = parser.close()
        cvImage = cv.CreateImageHeader(pilImage.size, cv.IPL_DEPTH_8U, 3)
        cv.SetData(cvImage, pilImage.tostring(), pilImage.size[0]*3)
        image = self.cvBridge.cv_to_imgmsg(cvImage, "bgr8")

    finally:
      cameraConnection.close()

    return image
  
  
def JoystickCallback(joystickMsg, manager):
  buttonIsPressed = joystickMsg.buttons[manager.buttonId] != 0
  if manager.buttonWasPressed:
    manager.buttonWasPressed = buttonIsPressed;
    return
  manager.buttonWasPressed = buttonIsPressed;

  if not buttonIsPressed:
    return

  # A new button press, so we need to capture a frame from the camera  
  image = manager.RetrieveImageFromCamera();
  if image is None:
    return
  manager.curImageId = manager.curImageId + 1

  # Now publish the image
  request = ImageCaptured(image_id=manager.curImageId, image=image)
  request.header.stamp = rospy.Time.now()
  manager.imagePublisher.publish(request)  
  

if __name__ == '__main__':
  rospy.init_node('CameraCaptureManager')
  manager = CameraCaptureManager()

  manager.Init()
  rospy.spin()
