#!/usr/bin/python
'''Process that watches the camera and restarts it if necessary

Author: Mark Desnoyer (markd@cmu.edu)
Date: Sept 2010
'''
import roslib; roslib.load_manifest('reefbot')
import rospy
import httplib
import math
import sys
import time
import urllib
import socket
import subprocess
from reefbot_msgs.msg import CameraHealth
from reefbot_msgs.msg import CameraPower
from reefbot_msgs.msg import VideoStream
from reefbot_msgs.msg import LogVideo
from std_msgs.msg import String

class ErrorCodes:
  OK = 0
  NO_CONNECT = 1
  NO_HTTP = 2

class CameraWatchdog:
  def __init__(self):
    # IP address of the camera
    self.cameraIp = rospy.get_param("camera_ip", "192.168.1.13")
    
    # Topic to publish the camera health on
    healthTopic = rospy.get_param("camera_health_topic", "camera_health")
    self.healthPublisher = rospy.Publisher(healthTopic, CameraHealth,
                                           tcp_nodelay=True,
                                           latch=True)
    self.cameraHealth = CameraHealth(ping_ok=False, http_ping_ok=False,
                                     error_code=ErrorCodes.NO_CONNECT)

    # Topic to send the camera power messages on
    powerTopic = rospy.get_param('camera_power_topic', 'camera_power')
    self.powerPublisher = rospy.Publisher(powerTopic, CameraPower,
                                          tcp_nodelay=True,
                                          latch=True)
    
    # Topic to send the video stream URL when waking up the camera
    videoTopic = rospy.get_param('video_stream_topic', 'video_stream')
    self.videoPublisher = rospy.Publisher(videoTopic, VideoStream,
                                          tcp_nodelay=True,
                                          latch=True)
    self.videoPublisher.publish(String(rospy.get_param("video_url", "")))

    # Topic to trigger a video logging
    videoLogTopic = rospy.get_param('log_video_topic', 'log_video')
    self.videoLogPub = rospy.Publisher(videoLogTopic, LogVideo,
                                       tcp_nodelay=True,
                                       latch=True)

    # Determine the ping rate in hertz
    self.pingRate = rospy.get_param('~ping_rate', 1.0)
    
    # Set the timeout for pings in seconds
    self.pingTimeout = rospy.get_param('~ping_timeout', 0.5)

    # Set the number of times we need to timeout before we try to power cycle
    self.pingRetries = rospy.get_param('~ping_retries', 2)

    # How long to wait for the camera to turn on an off (in seconds)
    self.cameraOffWait = rospy.get_param('~camera_off_wait', 0.5)
    self.cameraOnWait = rospy.get_param('~camera_on_wait', 10.0)

  def spin(self):
    rate = rospy.Rate(self.pingRate)
    while not rospy.is_shutdown():
      while not self.DoPing(self.DoICMPPing) and not rospy.is_shutdown():
        rospy.logerr('Cannot ping the camera at: %s' % self.cameraIp)
        self.cameraHealth.header.stamp = rospy.Time.now()
        self.healthPublisher.publish(self.cameraHealth)
        self.PowerCycleCamera()

      while not self.DoPing(self.DoImagePing) and not rospy.is_shutdown():
        self.cameraHealth.header.stamp = rospy.Time.now()
        self.healthPublisher.publish(self.cameraHealth)
        self.PowerCycleCamera()

      self.cameraHealth.error_code = ErrorCodes.OK

      self.cameraHealth.header.stamp = rospy.Time.now()
      self.healthPublisher.publish(self.cameraHealth)
      
      rate.sleep()

  def DoPing(self, pingFunc):
    '''Excutes multiple retries of a ping command and returns false if it fails.'''
    for i in range(self.pingRetries):
      if pingFunc():
        return True
    return False
             

  def DoICMPPing(self):
    '''Executes an ICMP ping with the camera.'''
    retCode = subprocess.call(["ping", '-c', '1', '-W',
                               str(math.ceil(self.pingTimeout)),
                               self.cameraIp])
    if retCode <> 0:
      self.cameraHealth.error_code = ErrorCodes.NO_CONNECT

    self.cameraHealth.ping_ok = retCode == 0

    return self.cameraHealth.ping_ok

  def DoImagePing(self):
    '''Pings the http channel on the camera by asking for a small image.'''
    try:
      # Make the connection
      cameraConnection = httplib.HTTPConnection(self.cameraIp,
                                                timeout=self.pingTimeout)
      cameraConnection.connect()

      # Ask for a small image
      try:
        cameraConnection.request("GET",
                                 ("/image?x0=0&y0=0&x1=32&y1=32&"
                                  "quality=18"))
        response = cameraConnection.getresponse()
      finally:
        cameraConnection.close()
    except httplib.HTTPException as e:
      self.FlagBadHttpConnection(e)
      return False
    except socket.error as e:
      self.FlagBadHttpConnection(e)
      return False

    self.cameraHealth.http_ping_ok = True;

    # See if there was an http error
    if response.status != 200:
      rospy.logerr('Received an error code from the camera %i, %s' %
                     (response.status, response.reason))
      self.cameraHealth.error_code = response.status
      return False

    return True 


  def FlagBadHttpConnection(self, e):
    rospy.logerr('Cannot connect to the camera(%s): %s' %
                 (self.cameraIp, e))
    self.cameraHealth.http_ping_ok = False;
    self.cameraHealth.error_code = ErrorCodes.NO_HTTP

  def PowerCycleCamera(self):
    # Turn off the camera and wait a bit
    rospy.logwarn('Turning off the camera')
    self.powerPublisher.publish(False)
    rospy.sleep(self.cameraOffWait)

    # Turn on the camera and ping until we get a response or the
    # timeout happens
    rospy.logwarn('Turning on the camera')
    self.powerPublisher.publish(True)
    timeWaited = 0;
    while not self.DoICMPPing() and timeWaited < self.cameraOnWait:
      timeWaited = timeWaited + self.pingTimeout
      
    if timeWaited >= self.cameraOnWait:
      rospy.logerr('Timed out waiting for a response from the camera '
                   'after a power cycle')
    else:
      rospy.loginfo('Camera is back on')
      self.videoPublisher.publish(String(
        rospy.get_param("video_url", ("http://192.168.1.13/mjpeg?"
                        "res=half&amp;x0=0&amp;y0=0&amp;x1=1920&amp;"
                        "y1=1080&amp;quality=10&amp;fps=30"))))
      
      self.videoLogPub.publish(
        String(rospy.get_param("video_bcast_ip", "239.255.15.42")),
        String(rospy.get_param("video_bcast_port", "5004")),
        -1) # Log duration

if __name__ == '__main__':
  rospy.init_node('CameraWatchdog')

  watchdog = CameraWatchdog()
  # Wait for the initial power spike to pass
  rospy.sleep(10.0)
  watchdog.spin()
